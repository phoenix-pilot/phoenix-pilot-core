// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// Linux headers
#include <errno.h>   // Error integer and strerror() function
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <time.h>
#include <sys/time.h>
#include <unistd.h> // write(), read(), close()

#include "gpsserver.h"

const char UART_DEVICE[] = "/dev/ttyUSB0";

/* types of messages interpreted, together with unknown, and broken message */
enum nmea_type {nmea_gga, nmea_gsa, nmea_rmc, nmea_vtg, nmea_unknown, nmea_broken};


/* GPGSA nmea type definitions: GPS DOP and active satellites */
typedef struct {
    unsigned int fix;
    float pdop;
    float hdop;
    float vdop;
} nmea_gsa_t;

enum nmea_fields_gsa {field_gsa_fix = 2, field_gsa_pdop = 15, field_gsa_hdop = 16, field_gsa_vdop = 17};


/* GPRMC nmea type definitions: Recommended minimum specific GPS/Transit data */
typedef struct {
    float lat;
    float lon;
    float speed;
    float course;
    float magvar;
} nmea_rmc_t;

enum nmea_fields_rmc {field_rmc_lat = 3, field_rmc_lon = 5, field_rmc_speedknots = 7, field_rmc_course = 8, field_rmc_magvar = 10};


/* GPVTG nmea type definitions: Track Made Good and Ground Speed. */
typedef struct {
    float track;
    char track_type;
    float speed_knots;
    float speed_kmh;
} nmea_vtg_t;

enum nmea_fields_vtg {field_vtg_track = 1, field_vtg_tracktype = 2, field_vtg_speedknots = 5, field_vtg_speedkmh = 7};


/* GPGGA nmea type definitions: Global Positioning System Fix Data  */
typedef struct {
    float lat;
    float lon;
    unsigned int fix;
    unsigned int sats;
    float hdop;
    float h_asl;
    float h_wgs;
} nmea_gga_t;

enum nmea_fields_gga {field_gga_lat = 2, field_gga_lon = 4, field_gga_fix = 6, field_gga_sats = 7, field_gga_hdop = 8, field_gga_h_asl = 9, field_gga_h_wgs = 11};


/* incoming message storage */
typedef struct {
    int type;
    union {
        nmea_gsa_t gsa;
        nmea_rmc_t rmc;
        nmea_vtg_t vtg;
        nmea_gga_t gga;
    } msg;

} nmea_t;


static gps_data_t gps_data = {0};
static unsigned int gps_data_fresh = 0;


/* moves pointer to the comma character just before the n-th field of nmea message in str. Returns pointer new value or NULL if search for n-th field failed */
static char * nmea_nField(char ** pointer, char * str, int n)
{
    for (int i = 0; i < n && str != NULL; i++ ) {
        str = (*(str + sizeof(char)) == ',') ? str + sizeof(char) : strchr(str + 1, ',');
    }
    *pointer = str;
    return *pointer;
}


static int parse_nmea_gsa(char * str, nmea_t * out)
{
    char * p;
    nmea_gsa_t in;

    do {
        if (nmea_nField(&p, str, field_gsa_fix) == NULL) {
            break;
        }
        in.fix = strtoul(p + 1, NULL, 10);
        if (in.fix == 0 || in.fix > 3) {
            break;
        }

        if (nmea_nField(&p, str, field_gsa_pdop) == NULL) {
            break;
        }
        in.pdop = strtod(p + 1, NULL);

        if (nmea_nField(&p, str, field_gsa_hdop) == NULL) {
            break;
        }
        in.hdop = strtod(p + 1, NULL);

        if (nmea_nField(&p, str, field_gsa_vdop) == NULL) {
            break;
        }
        in.vdop = strtod(p + 1, NULL);

        out->msg.gsa = in;
        out->type = nmea_gsa;
        return nmea_gsa;

    } while (0);

    return nmea_broken;
}


static int parse_nmea_vtg(char * str, nmea_t * out)
{
    char * p;
    nmea_vtg_t in;

    do {
        if (nmea_nField(&p, str, field_vtg_track) == NULL) {
            break;
        }
        in.track = strtoul(p + 1, NULL, 10);

        if (nmea_nField(&p, str, field_vtg_speedknots) == NULL) {
            break;
        }
        in.speed_knots = strtod(p + 1, NULL);

        if (nmea_nField(&p, str, field_vtg_speedkmh) == NULL) {
            break;
        }
        in.speed_kmh = strtod(p + 1, NULL);

        out->msg.vtg = in;
        out->type = nmea_vtg;
        return nmea_vtg;

    } while (0);

    return nmea_broken;
}


static int parse_nmea_gga(char * str, nmea_t * out)
{
    char * p, * sign;
    nmea_gga_t in;

    do {
        /* latitude read */
        if (nmea_nField(&p, str, field_gga_lat) == NULL || nmea_nField(&sign, str, field_gga_lat + 1) == NULL) {
            break;
        }
        in.lat = strtod(p + 1, NULL);
        in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
        in.lat = (*(sign + 1) == 'S') ? -in.lat : in.lat;

        /* longitude read */
        if (nmea_nField(&p, str, field_gga_lon) == NULL || nmea_nField(&sign, str, field_gga_lon + 1) == NULL) {
            break;
        }
        in.lon = strtod(p + 1, NULL);
        in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
        in.lon = (*(sign + 1) == 'S') ? -in.lon : in.lon;

        /* GPS Quality indicator read */
        if (nmea_nField(&p, str, field_gga_fix) == NULL) {
            break;
        }
        in.fix = strtoul(p + 1, NULL, 10);
        if (in.fix == 0 || in.fix > 3) {
            break;
        }

        /* number of satellites in use read */
        if (nmea_nField(&p, str, field_gga_sats) == NULL) {
            break;
        }
        in.sats = strtoul(p + 1, NULL, 10);

        /* horizontal dilution of precision read */
        if (nmea_nField(&p, str, field_gga_hdop) == NULL) {
            break;
        }
        in.hdop = strtod(p + 1, NULL);

        /* Antenna altitude above mean-sea-level read */
        if (nmea_nField(&p, str, field_gga_h_asl) == NULL) {
            break;
        }
        in.h_asl = strtod(p + 1, NULL);

        /* Geoidal separation read */
        if (nmea_nField(&p, str, field_gga_h_wgs) == NULL) {
            break;
        }
        in.h_wgs = strtod(p + 1, NULL);

        out->msg.gga = in;
        out->type = nmea_gga;
        return nmea_gga;

    } while (0);

    return nmea_broken;
}


static int parse_nmea_rmc(char * str, nmea_t * out)
{
    char * p, * sign;
    nmea_rmc_t in;

    do {
        /* latitude read */
        if (nmea_nField(&p, str, field_rmc_lat) == NULL || nmea_nField(&sign, str, field_rmc_lat + 1) == NULL) {
            break;
        }
        in.lat = strtod(p + 1, NULL);
        in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
        in.lat = (*(sign + 1) == 'S') ? -in.lat : in.lat;

        /* longitude read */
        if (nmea_nField(&p, str, field_rmc_lon) == NULL || nmea_nField(&sign, str, field_rmc_lon + 1) == NULL) {
            break;
        }
        in.lon = strtod(p + 1, NULL);
        in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
        in.lon = (*(sign + 1) == 'S') ? -in.lon : in.lon;

        /* speed in knots read */
        if (nmea_nField(&p, str, field_rmc_speedknots) == NULL) {
            break;
        }
        in.speed = strtod(p + 1, NULL);

        /* course read */
        if (nmea_nField(&p, str, field_rmc_course) == NULL) {
            break;
        }
        in.course = strtod(p + 1, NULL);

        /* magnetic variation read */
        if (nmea_nField(&p, str, field_rmc_magvar) == NULL) {
            break;
        }
        in.magvar = strtod(p + 1, NULL);

        out->msg.rmc = in;
        out->type = nmea_rmc;
        return nmea_rmc;

    } while (0);

    return nmea_broken;
}


/* interpret one line of gps outpu into nmea message, if known */
static int nmeainterpreter_string(char * str, nmea_t * out)
{
    if (strncmp(str, "$GPGSA", 6) == 0) {
        return parse_nmea_gsa(str, out);
    }

    if (strncmp(str, "$GPVTG", 6) == 0) {
        return parse_nmea_vtg(str, out);
    }
    
    if (strncmp(str, "$GPGGA", 6) == 0) {
        return parse_nmea_gga(str, out);
    }

    if (strncmp(str, "$GPRMC", 6) == 0) {
        return parse_nmea_rmc(str, out);
    }

    return nmea_unknown;
}


static int nmea_update(nmea_t * message, gps_data_t * gps_data)
{
    switch (message->type) {
        case nmea_gga:
            gps_data->lat = message->msg.gga.lat * 1e7;
            gps_data->lon = message->msg.gga.lon * 1e7;
            gps_data->hdop = (unsigned int)(message->msg.gga.hdop * 1e2);
            //gps_data->fix = message->msg.gga.fix;
            gps_data->alt = message->msg.gga.h_asl * 1e3;
            gps_data->altEllipsoid = message->msg.gga.h_wgs * 1e3;
            gps_data->satsNb = message->msg.gga.sats;
            break;
        
        case nmea_gsa:
            gps_data->hdop = (unsigned int)(message->msg.gsa.hdop * 1e2);
            gps_data->vdop = (unsigned int)(message->msg.gsa.vdop * 1e2);
            break;

        case nmea_rmc:
            break;

        case nmea_vtg:
            gps_data->heading = message->msg.vtg.track * 0.017453 * 1e3; /* degrees -> milliradians */
            gps_data->groundSpeed = message->msg.vtg.speed_kmh * 1e6 * 0.000277778; /* kmh->mm/s */
            gps_data->velNorth = cos(message->msg.vtg.track * 0.017453) * gps_data->groundSpeed;
            gps_data->velEast = sin(message->msg.vtg.track * 0.017453) * gps_data->groundSpeed;
            break;

        default:
            break;
    }
    return 0;
}

static int getlines(char ** buf) {
    int n = 0;
    char *start, *curr;
    
    start = strchr(*buf, '$');
    if (start == NULL) {
        return 0;
    }

    curr = start;
    do {
        curr = strchr(curr, '$');
        if (curr != NULL) {
            curr = strchr(curr, '*');
        }

        n++;
    } while (curr != NULL); 
    n--;

    if (n > 0) {
        *buf = start;
    }
    return n;
    
}

static int nmeainterpretter_filedes(int filedes)
{
    char buf[1024], *start;
    int n, ret;
    nmea_t message = { 0 };

    while (1) {
            memset(buf, 0, sizeof(buf));
            read(filedes, buf, sizeof(buf));
            sleep(2);
            start = buf;
            n = getlines(&start);
            while (n > 0) {
                ret = nmeainterpreter_string(start, &message);
                if(ret != nmea_broken && ret != nmea_unknown) {
                    nmea_update(&message, &gps_data);
                }
                n--;
                start = strchr(start + 1, '$');
            }
            gps_data_fresh = 1;
    }
    return 0;
}

void gpsServerThread(void)
{
    int fd;

    fd = open("/dev/uart2", O_RDONLY);
    if (fd >= 0) {
        nmeainterpretter_filedes(fd);
        close(fd);
    }
    printf("failed to open uart2\n");
}

void print_gps_data(gps_data_t * data)
{
    printf("lat/lon | hdop:\t%d/%d | %d\n", data->lat, data->lon, data->hdop);
    printf("asl/wgs | vdop:\t%d/%d | %d\n", data->alt, data->altEllipsoid, data->vdop);
    printf("kmh/kmhN/kmhE:\t%d/%d/%d\n", data->groundSpeed, data->velNorth, data->velEast);
    printf("hdop/vdop/sats:\t%d/%d/%d\n", data->hdop, data->vdop, data->satsNb);
    printf("\n");
}


int sensGps(gps_data_t * data)
{
    if (gps_data_fresh > 0) {
        *data = gps_data;
        gps_data_fresh = 0;
        return 2;
    }
    return 0;
}
