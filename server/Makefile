# Makefile to install MuM components

WEBROOT=/fs1/www/ajf/dl1.findlays.net

all:
	echo Nothing needs building here...

install:
	install -c -m 755 influxdb-backup $(WEBROOT)/bin/influxdb-backup
	install -c -m 755 mumdata $(WEBROOT)/wsgi/mumdata
	if test ! -f $(WEBROOT)/etc/mumdata.conf; then install -c -m 640 -o ajf -g www-data mumdata.conf $(WEBROOT)/etc/mumdata.conf; fi
	install -c -m 755 mqtt-to-influxdb $(WEBROOT)/libexec/mqtt-to-influxdb
	install -c -m 644 mqtt-to-influxdb.service /etc/systemd/system/mqtt-to-influxdb.service
	if test ! -f $(WEBROOT)/etc/mqtt-to-influxdb.conf; then install -c -m 640 -o ajf -g www-data mqtt-to-influxdb.conf $(WEBROOT)/etc/mqtt-to-influxdb.conf; fi
	systemctl daemon-reload
	systemctl enable mqtt-to-influxdb

