
#!/bin/sh
/sbin/route add default gw 192.168.7.1

echo "nameserver 192.168.7.1

# Google IPv4 nameservers
nameserver 8.8.8.8
nameserver 8.8.4.4
# Google IPv6 nameservers
nameserver 2001:4860:4860::8888
nameserver 2001:4860:4860::8844" > /etc/resolv.conf

echo "Internet connection complete"
