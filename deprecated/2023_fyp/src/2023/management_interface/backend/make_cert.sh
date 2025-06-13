#!/bin/bash
mkdir -p certs
openssl req \
  -newkey rsa:4096 -nodes -sha256 -keyout $CERT_PATH/domain.key \
  -addext "subjectAltName = IP:$IP" \
  -subj "/C=NZ/ST=Canterbury/L=Christchurch/O=UC/OU=CSSE/CN=$IP" \
  -x509 -days 365 -out "$CERT_PATH"/domain.crt

mkdir -p /etc/docker/certs.d/drones
cp "$CERT_PATH"/domain.crt /etc/docker/certs.d/drones/ca.crt
cp "$CERT_PATH"/domain.crt /usr/local/share/ca-certificates/drones.crt
update-ca-certificates
systemctl restart docker