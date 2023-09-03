#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include <linux/dvb/dmx.h>
#include <linux/dvb/frontend.h>

static int main_adapter_idx;

static int (*libc_close)(int fd);
static int (*libc_open)(const char *pathname, int flags, ...);
static int (*libc_open64)(const char *pathname, int flags, ...);
static int (*libc_stat)(int ver, const char *path, struct stat *s);
static int (*libc_stat64)(int ver, const char *path, struct stat *s);

#define ARRAY_SIZE(arr) \
    (sizeof(arr) / sizeof((arr)[0]) \
     + sizeof(typeof(int[1 - 2 * \
           !!__builtin_types_compatible_p(typeof(arr), \
                 typeof(&arr[0]))])) * 0)

#define VTUNER_FLUSH 0
#define VTUNER_GET_MESSAGE  1
#define VTUNER_SET_RESPONSE 2
#define VTUNER_SET_NAME     3
#define VTUNER_SET_TYPE     4
#define VTUNER_SET_HAS_OUTPUTS 5
#define VTUNER_SET_FE_INFO  6
#define VTUNER_SET_NUM_MODES   7
#define VTUNER_SET_MODES   8

#define MSG_PIDLIST              14

struct dvb_frontend_tune_settings {
	int min_delay_ms;
	int step_size;
	int max_drift;
	struct dvb_frontend_parameters parameters;
};

struct vtuner_message
{
	__s32 type;
	union
	{
		struct dvb_frontend_parameters dvb_frontend_parameters;
		struct dvb_frontend_tune_settings dvb_frontend_tune_settings;
		struct dtv_property prop;
		fe_status_t status;
		__u32 ber;
		__u16 ss, snr;
		__u32 ucb;
		fe_sec_tone_mode_t tone;
		fe_sec_voltage_t voltage;
		struct dvb_diseqc_master_cmd diseqc_master_cmd;
		fe_sec_mini_cmd_t burst;
		__u16 pidlist[36];
		unsigned char pad[72];
		__u32 type_changed;
	} body;
};

#define MAX_ADAPTERS 8
#define BUFFER_SIZE ((188 / 4) * 4096) /* multiple of ts packet and page size */
#define DEMUX_BUFFER_SIZE (8 * ((188 / 4) * 4096)) /* 1.5MB */

struct vtuner_adapter
{
	int idx;    /* dvb adapter index */
	char name[64];
	char *buffer;
	int frontend; /* fd of usb tuner frontend device */
	int demux;    /* fd of usb tuner demux device */
	int vtuner;   /* fd of vtuner device */
	int vtuneridx; /* vtuner index */
	volatile int running;

	int vtunerfeidx; /* index of frontend device created by associated vtuner */

	volatile pthread_t eventthread;
	volatile pthread_t pumpthread;

	__u16 pidlist[36];
};

struct vtuner_adapter adapters[MAX_ADAPTERS];
static int assigned_adapters;

static int scan_adapters()
{
	char filename[256];
	int i;
	int created = 0, found = 0;
	int frontends_before = 0;

	while (1)
	{
		char tmp[] = "/sys/class/dvb/dvb0.frontend0/dev";
		tmp[18] = '0' + main_adapter_idx;
		tmp[28] = '0' + frontends_before;
		if (access(tmp, F_OK))
			break;
		++frontends_before;
	}

	int nr = 0;
	while (found < MAX_ADAPTERS)
	{
		FILE *fd;
		if (nr != main_adapter_idx)
		{
			snprintf(filename, sizeof(filename), "/sys/class/dvb/dvb%d.frontend0/device/product", nr);
			fd = fopen(filename, "r");
			if (!fd)
			{
				snprintf(filename, sizeof(filename), "/sys/class/dvb/dvb%d.frontend0/device/manufacturer", nr);
				fd = fopen(filename, "r");
			}

			if (fd)
			{
				char *tmp = adapters[found].name;
				fread(tmp, 63, 1, fd);
				tmp[63] = 0;
				while (strlen(tmp) > 0 && (tmp[strlen(tmp) - 1] == '\n' || tmp[strlen(tmp) - 1] == ' '))
					tmp[strlen(tmp) - 1] = 0;
				fclose(fd);
			}
			else {
				strcpy(adapters[found].name, "unknown frontend");
				break;
			}

			adapters[found].idx = nr;
			++found;
		}
		++nr;
	}

	for (i = 0; i < found; i++)
	{
		int vtuner=0;
		char tmp[] = "/dev/misc/vtuner0";
		for (vtuner=0; vtuner < 8; ++vtuner)
		{
			tmp[16] = '0' + i;
			adapters[i].vtuner = libc_open(tmp, O_RDWR);
			if (adapters[i].vtuner < 0)
			{
				if (errno == -EBUSY) // skip busy vtuners
					continue;
				break;
			}
			adapters[i].vtuneridx = i;
			adapters[i].vtunerfeidx = frontends_before + i;
			printf("USB Tuner '%s' adapter%d assigned to vtuner%d frontend%d\n", adapters[i].name, adapters[i].idx, i, frontends_before + i);

			++created;
			break;
		}

		if (adapters[i].vtuner < 0)
			printf("USB Tuner '%s' adapter%d could not be assigned ... all vtuners busy!\n", adapters[i].name, adapters[i].idx);
	}

	return created;
}

static void *event_task(void *ptr)
{
	int i, j;
	struct vtuner_adapter *adapter = (struct vtuner_adapter*)ptr;

	while (adapter->running)
	{
		int ret;
		struct pollfd pfd;
		pfd.fd = adapter->vtuner;
		pfd.events = POLLPRI;

		ret = poll(&pfd, 1, 1000);
		if (ret > 0)
		{
			struct vtuner_message message;

			if (ioctl(adapter->vtuner, VTUNER_GET_MESSAGE, &message) < 0) {
				if (errno != EINTR) {
					fprintf(stderr, "USB Tuner '%s' VTUNER_GET_MESSAGE failed (%m)\n", adapter->name);
					adapter->running = 0;
					break;
				}
				continue;
			}

			switch (message.type)
			{
			case MSG_PIDLIST:
			{
				int active_pids = 0;

				/* remove old pids */
				for (i = 0; i < ARRAY_SIZE(message.body.pidlist); i++)
				{
					if (adapter->pidlist[i] != 0xffff)
					{
						int found = 0;

						for (j = 0; j < ARRAY_SIZE(message.body.pidlist); j++)
						{
							if (adapter->pidlist[i] == message.body.pidlist[j])
							{
								found = 1;
								break;
							}
						}

						if (!found)
						{
							printf("DMX_REMOVE_PID %x\n", adapter->pidlist[i]);
							ioctl(adapter->demux, DMX_REMOVE_PID, &adapter->pidlist[i]);
						}
					}
				}

				/* add new pids */
				for (i = 0; i < ARRAY_SIZE(message.body.pidlist); i++)
				{
					if (message.body.pidlist[i] != 0xffff)
					{
						int found = 0;

						for (j = 0; j < ARRAY_SIZE(message.body.pidlist); j++)
						{
							if (message.body.pidlist[i] == adapter->pidlist[j])
							{
								found = 1;
								break;
							}
						}

						if (!found)
						{
							printf("DMX_ADD_PID %x\n", message.body.pidlist[i]);
							ioctl(adapter->demux, DMX_ADD_PID, &message.body.pidlist[i]);
						}
					}
				}

				/* copy pids */
				for (i = 0; i < ARRAY_SIZE(message.body.pidlist); i++)
				{
					adapter->pidlist[i] = message.body.pidlist[i];

					if (adapter->pidlist[i] != 0xffff)
						++active_pids;
				}

				if (!active_pids) {

				}

				break;
			}
			default:
				printf("Unhandled vtuner message type: %d\n", message.type);
				break;
			}

			if (message.type != MSG_PIDLIST)
			{
				message.type = 0;
				if (ioctl(adapter->vtuner, VTUNER_SET_RESPONSE, &message) < 0)
					perror("ioctl VTUNER_SET_RESPONSE");
			}
		}
	}

error:
	return NULL;
}

static void *data_task(void *ptr)
{
	struct vtuner_adapter *adapter = (struct vtuner_adapter *)ptr;
	char demux_filename[256];
	struct dmx_pes_filter_params filter;

	sprintf(demux_filename, "/dev/dvb/adapter%d/demux0", adapter->idx);

	adapter->demux = libc_open(demux_filename, O_RDONLY | O_NONBLOCK);
	if (adapter->demux < 0)
	{
		adapter->running = 0;
		perror(demux_filename);
		printf("USB Tuner '%s' failed to start thread\n", adapter->name);
		return NULL;
	}

	printf("USB Tuner '%s' pump thread running\n", adapter->name);

	filter.input = DMX_IN_FRONTEND;
	filter.flags = 0;
	filter.pid = 0;
	filter.output = DMX_OUT_TSDEMUX_TAP;
	filter.pes_type = DMX_PES_OTHER;

	ioctl(adapter->demux, DMX_SET_BUFFER_SIZE, DEMUX_BUFFER_SIZE);
	ioctl(adapter->demux, DMX_SET_PES_FILTER, &filter);
	ioctl(adapter->demux, DMX_START);

	if (pthread_create((pthread_t*)&adapter->eventthread, NULL, event_task, (void*)adapter)) {
		printf("USB Tuner '%s' failed to create event thread\n", adapter->name);
		libc_close(adapter->demux);
		adapter->demux = -1;
		adapter->eventthread = 0;
		adapter->running = 0;
	}

	while (adapter->running)
	{
		int ret;
		struct pollfd pfd;
		pfd.fd = adapter->demux;
		pfd.events = POLLIN;

		ret = poll(&pfd, 1, 1000);
		if (ret > 0)
		{
			ssize_t rd = read(adapter->demux, adapter->buffer, BUFFER_SIZE);
			ssize_t written = 0;
			if (rd < 0)
			{
				if (errno == EINTR)
					continue;
				fprintf(stderr, "USB Tuner '%s' demux read failed (%m)\n", adapter->name);
				goto error_out;
			}
			else while (written < rd)
			{
				ssize_t wr = write(adapter->vtuner, adapter->buffer + written, rd - written);
				if (wr < 0)
				{
					if (errno == EINTR)
						continue;
					fprintf(stderr, "USB Tuner '%s' vtuner write failed (%m)\n", adapter->name);
					goto error_out;
				}

				written += wr;
			}
		}
		else if (ret < 0)
		{
			if (errno != EINTR)
			{
				fprintf(stderr, "usb tuner%d poll failed (%m)", adapter->idx);
				goto error_out;
			}
		}
	}

error_out:
	if (adapter->running)
	{
		printf("USB Tuner '%s' pump thread aborted!\n", adapter->name);
		adapter->running = 0;
	}

	pthread_join(adapter->eventthread, NULL);

	if (adapter->demux != -1) {
		libc_close(adapter->demux);
		adapter->demux = -1;
	}

	return NULL;
}

#ifndef DTV_ENUM_DELSYS
    #define DTV_ENUM_DELSYS 44
    #define SYS_DVBC_ANNEX_A SYS_DVBC_ANNEX_AC
#endif

static int init_adapter(int id)
{
	char type[8];
	char frontend_filename[256];
	struct vtuner_adapter *adapter = &adapters[id];

	adapter->eventthread = adapter->pumpthread = 0;

	sprintf(frontend_filename, "/dev/dvb/adapter%d/frontend0", adapter->idx);

	adapter->frontend = adapter->demux = -1;

	adapter->frontend = libc_open(frontend_filename, O_RDWR);
	if (adapter->frontend < 0)
	{
		perror(frontend_filename);
		goto error;
	}

	if (ioctl(adapter->vtuner, VTUNER_SET_NAME, adapter->name) < 0)
		perror("ioctl VTUNER_SET_NAME");

	{
		struct dtv_properties props;
		struct dtv_property p[1];
		props.num = 1;
		props.props = p;
		p[0].cmd = DTV_ENUM_DELSYS;

		if (ioctl(adapter->frontend, FE_GET_PROPERTY, &props) >= 0)
		{
			char modes[3][32];
			int i=0;
			int mode_mask=0;
			for (; i < p[0].u.buffer.len; ++i)
			{
				switch(p[0].u.buffer.data[i]) {
					case SYS_DVBS: mode_mask |= 1; break;
					case SYS_DVBS2: mode_mask |= 2; break;
					case SYS_DVBC_ANNEX_A: mode_mask |= 4; break;
					case SYS_DVBT: mode_mask |= 8; break;
					case SYS_DVBT2: mode_mask |= 16; break;
					default: break;
				}
			}

			memset(modes, 0, sizeof(modes));

			if (mode_mask & 2)
				mode_mask &= ~1;
			if (mode_mask & 16)
				mode_mask &= ~8;

			unsigned long num_modes=0;

			while (mode_mask && num_modes < 3) {
				if (mode_mask & 1) {
					strcpy(modes[num_modes], "DVB-S");
					mode_mask &= ~1;
				}
				else if (mode_mask & 2) {
					strcpy(modes[num_modes], "DVB-S2");
					mode_mask &= ~2;
				}
				else if (mode_mask & 4) {
					strcpy(modes[num_modes], "DVB-C");
					mode_mask &= ~4;
				}
				else if (mode_mask & 8) {
					strcpy(modes[num_modes], "DVB-T");
					mode_mask &= ~8;
				}
				else if (mode_mask & 16) {
					strcpy(modes[num_modes], "DVB-T2");
					mode_mask &= ~16;
				}
				++num_modes;
			}

			if (num_modes == 0)
				goto error;

			if (num_modes > 1)
			{
				if (ioctl(adapter->vtuner, VTUNER_SET_NUM_MODES, num_modes) < 0)
					perror("ioctl VTUNER_SET_NUM_MODES");
				if (ioctl(adapter->vtuner, VTUNER_SET_MODES, modes) < 0)
					perror("ioctl VTUNER_SET_MODES");
			}
			else
			{
				if (ioctl(adapter->vtuner, VTUNER_SET_TYPE, modes) < 0)
					perror("ioctl VTUNER_SET_TYPE");
			}
		}
	}

	if (ioctl(adapter->vtuner, VTUNER_SET_HAS_OUTPUTS, "no") < 0)
		perror("ioctl VTUNER_SET_HAS_OUTPUTS");

	memset(adapter->pidlist, 0xff, sizeof(adapter->pidlist));
	adapter->buffer = malloc(BUFFER_SIZE);

	libc_close(adapter->frontend);
	adapter->frontend = -1;

	return 0;

error:
	if (adapter->vtuner >= 0)
	{
		libc_close(adapter->vtuner);
		adapter->vtuner = -1;
	}
	if (adapter->demux >= 0)
	{
		libc_close(adapter->demux);
		adapter->demux = -1;
	}
	if (adapter->frontend >= 0)
	{
		libc_close(adapter->frontend);
		adapter->frontend = -1;
	}

	return -1;
}

static void scan_usb_tuners()
{
	int i, n;
	int entries[MAX_ADAPTERS];
	struct dirent **names;

	memset(entries, 0, sizeof(entries));

	n = scandir("/sys/class/dvb", &names, NULL, NULL);
	if (n > 0)
	{
		while (n--)
		{
			if (names[n]->d_name[0] == '.')
				;
			else
			{
				int adap = names[n]->d_name[3] - '0';
				if (adap < MAX_ADAPTERS)
					++entries[adap];
				else
					fprintf(stderr, "skipped '%s' on main adapter lookup\n", names[n]->d_name);
			}
			free(names[n]);
		}
		free(names);
	}

	n = 0;
	for (i = 0; i < MAX_ADAPTERS; ++i)
	{
		if (entries[i] > n)
		{
			n = entries[i];
			main_adapter_idx = i;
		}
	}

	printf("Detected DVB adapter%d as main adapter\n", main_adapter_idx);

	for (i = 0; i < MAX_ADAPTERS; ++i)
		adapters[i].vtuner = -1;

	printf("Scanning for USB Tuners\n");

	assigned_adapters = scan_adapters();

	for (i = 0; i < assigned_adapters; i++)
	{
		if (adapters[i].vtuner >= 0)
		{
			if (init_adapter(i) < 0)
			{
				libc_close(adapters[i].vtuner);
				adapters[i].vtuner = -1;
			}
		}
	}
}

static int check_usb_tuner_open(const char *path, int flags, mode_t mode)
{
	int i;
	char fe[] = "/dev/dvb/adapter0/frontend0";
	fe[16] = '0' + main_adapter_idx;

	for (i = 0; i < assigned_adapters; i++)
	{
		if (adapters[i].vtuner >= 0)
		{
			fe[26] = '0' + adapters[i].vtunerfeidx;
			if (!strcmp(path, fe))
			{
				char frontend_filename[32];
				printf("USB Tuner '%s' open (%s)\n", adapters[i].name, path);

				sprintf(frontend_filename, "/dev/dvb/adapter%d/frontend0", adapters[i].idx);

				adapters[i].frontend = libc_open(frontend_filename, flags, mode);
				if (adapters[i].frontend < 0)
					perror(frontend_filename);
				else
				{
					adapters[i].eventthread = adapters[i].pumpthread = 0;
					adapters[i].running = 1;

					if (pthread_create((pthread_t*)&adapters[i].pumpthread, NULL, data_task, (void*)adapters+i))
					{
						printf("USB Tuner '%s' failed to create pump thread\n", adapters[i].name);
						adapters[i].running = 0;
					}

					while(adapters[i].running && !adapters[i].eventthread);

					if (adapters[i].running)
							printf("USB Tuner '%s' ready now\n", adapters[i].name);
					else {
						libc_close(adapters[i].frontend);
						adapters[i].frontend = -1;

						errno = ENOMEM;
						return -3;
					}

					return adapters[i].frontend;
				}

				printf("USB Tuner '%s' open failed (%m)\n", adapters[i].name);

				return adapters[i].frontend;
			}
		}
	}

	return -1;
}

static int check_usb_tuner_close(int fd)
{
	int i;
	char fe[] = "/dev/dvb/adapter0/frontend0";
	fe[16] = '0' + main_adapter_idx;

	for (i = 0; i < assigned_adapters; i++)
	{
		if (adapters[i].vtuner >= 0)
		{
			if (adapters[i].frontend == fd)
			{
				fe[26] = '0' + adapters[i].vtunerfeidx;

				printf("USB Tuner '%s' close request (%s)\n", adapters[i].name, fe);

				adapters[i].running = 0;

				pthread_join((pthread_t)adapters[i].pumpthread, NULL);

				printf("USB Tuner '%s' closed\n", adapters[i].name);

				adapters[i].frontend = -1;

				return libc_close(fd);
			}
		}
	}

	return -1;
}

static int check_access(const char *p)
{
	int i;
	for (i = 0; i < assigned_adapters; i++)
	{
		if (adapters[i].vtuner >= 0)
		{
			char frontend_filename[32];
			sprintf(frontend_filename, "/dev/dvb/adapter%d/frontend0", adapters[i].idx);

			if (!strncmp(p, frontend_filename, 17))
				return -1;
		}
	}
	return 0;
}

extern void *find_symbol(void *hdn, const char *symbol, void *repl);

static void initialize_globals_ctor(void) __attribute__ ((constructor));
static void initialize_globals(void);

#define CALL(func, ...) __extension__ \
({ \
	if (!func) \
		initialize_globals(); \
	func(__VA_ARGS__); \
})

static void initialize_globals_ctor(void)
{
	initialize_globals();
}

static int libusbtuner_close(int fd)
{
	int ret;

	if (!libc_close)
		initialize_globals();

	if (fd >= 0)
	{
		ret = check_usb_tuner_close(fd);
		if (ret == 0)
			return ret;
	}

	return libc_close(fd);
}

#define HANDLE_PATH(in, err) \
	const char *path = NULL; \
	char tmp[] = "/dev/dvb/adapter0/              "; \
	if (assigned_adapters && main_adapter_idx) \
	{ \
		if (!strncmp(in, tmp, 17)) \
		{ \
			strncpy(tmp, in, 32); \
			tmp[16] = '0' + main_adapter_idx; \
			path = tmp; \
		} \
		if (!path) \
		{ \
			tmp[16] = '0' + main_adapter_idx; \
			if (!strncmp(in, tmp, 17)) \
			{ \
				strncpy(tmp, in, 32); \
				tmp[16] = '0'; \
				path = tmp; \
			} \
		} \
	} \
	if (!path) \
		path = in; \
	else if (assigned_adapters && check_access(path) < 0) \
	{ \
		errno = err; \
		return -1; \
	}

static int libusbtuner_open64(const char *pathname, int flags, ...)
{
	mode_t mode;
	va_list ap;
	int fd;
	HANDLE_PATH(pathname, EBUSY);

	va_start(ap, flags);
	mode = va_arg(ap, mode_t);
	va_end(ap);

	if (!libc_open64)
		initialize_globals();

	fd = check_usb_tuner_open(path, flags, mode);
	if (fd == -1)
		fd = libc_open64(path, flags, mode);

	return fd;
}

int libusbtuner_open(const char *pathname, int flags, ...)
{
	mode_t mode;
	va_list ap;
	int fd;
	HANDLE_PATH(pathname, EBUSY);

	va_start(ap, flags);
	mode = va_arg(ap, mode_t);
	va_end(ap);

	if (!libc_open)
		initialize_globals();

	fd = check_usb_tuner_open(path, flags, mode);

	if (fd < 0)
		fd = libc_open(path, flags, mode);

	return fd;
}

int libusbtuner_stat(int ver, const char * p, struct stat *s)
{
	HANDLE_PATH(p, EACCES);

	return CALL(libc_stat, ver, path, s);
}

int libusbtuner_stat64(int ver, const char *p, struct stat *s)
{
	HANDLE_PATH(p, EACCES);

	return CALL(libc_stat64, ver, path, s);
}

static void initialize_globals(void)
{
	FILE *f;
	char tmp[255];

	if (libc_close)
		return;

	libc_close = find_symbol(NULL, "close", libusbtuner_close);
	libc_open = find_symbol(NULL, "open", libusbtuner_open);
	libc_open64 = find_symbol(NULL, "open64", libusbtuner_open64);
	libc_stat = find_symbol(NULL, "__xstat", libusbtuner_stat);
	libc_stat64 = find_symbol(NULL, "__xstat64", libusbtuner_stat64);

	snprintf(tmp, sizeof(tmp), "/proc/%d/cmdline", getpid());
	f = fopen(tmp, "r");
	if (f)
	{
		ssize_t rd = fread(tmp, 1, 255, f);
		if (rd > 0)
		{
			const char *s = strstr(tmp, "enigma2");
			if ((s == tmp || (s && s[-1] == '/')) && !s[7]) /* check if enigma2 or /enigma2 */
				scan_usb_tuners();
		}
		fclose(f);
	}
}

int open(const char *, int, ...) __attribute__ ((weak, alias ("libusbtuner_open")));
int open64(const char *, int, ...) __attribute__ ((weak, alias ("libusbtuner_open64")));
int close(int) __attribute__ ((weak, alias ("libusbtuner_close")));
int __xstat(int, const char *, struct stat *) __attribute__ ((weak, alias ("libusbtuner_stat")));
int __xstat64(int, const char *, struct stat *) __attribute__ ((weak, alias ("libusbtuner_stat64")));
