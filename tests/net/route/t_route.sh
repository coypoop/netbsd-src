#	$NetBSD: t_route.sh,v 1.4 2016/04/04 07:37:08 ozaki-r Exp $
#
# Copyright (c) 2016 Internet Initiative Japan Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

RUMP_LIBS="-lrumpnet -lrumpnet_net -lrumpnet_netinet -lrumpnet_shmif"
RUMP_LIBS_IPV6="$RUMP_LIBS -lrumpnet_netinet6"

# non_subnet_gateway
SOCK_CLIENT=unix://commsock1
SOCK_GW=unix://commsock2
BUS=bus1

# command_get
SOCKSRC=unix://commsock1
SOCKFWD=unix://commsock2
SOCKDST=unix://commsock3
IP4SRC=10.0.1.2
IP4SRCGW=10.0.1.1
IP4DSTGW=10.0.2.1
IP4DST=10.0.2.2
IP4DST_BCAST=10.0.2.255
IP6SRC=fc00:0:0:1::2
IP6SRCGW=fc00:0:0:1::1
IP6DSTGW=fc00:0:0:2::1
IP6DST=fc00:0:0:2::2
BUS_SRCGW=bus1
BUS_DSTGW=bus2

DEBUG=false
TIMEOUT=1
PING_OPTS="-n -c 1 -w $TIMEOUT"

atf_test_case non_subnet_gateway cleanup
non_subnet_gateway_head()
{

	atf_set "descr" "tests of a gateway not on the local subnet"
	atf_set "require.progs" "rump_server"
}

non_subnet_gateway_body()
{

	atf_check -s exit:0 rump_server ${RUMP_LIBS} ${SOCK_CLIENT}
	atf_check -s exit:0 rump_server ${RUMP_LIBS} ${SOCK_GW}

	export RUMP_SERVER=${SOCK_GW}

	atf_check -s exit:0 rump.ifconfig shmif0 create
	atf_check -s exit:0 rump.ifconfig shmif0 linkstr $BUS
	atf_check -s exit:0 rump.ifconfig shmif0 192.168.0.1
	atf_check -s exit:0 rump.ifconfig shmif0 up

	# The gateway knows the client
	atf_check -s exit:0 -o match:'add net 10.0.0.1: gateway shmif0' \
	    rump.route add -net 10.0.0.1/32 -link -cloning -iface shmif0

	$DEBUG && rump.netstat -nr -f inet

	export RUMP_SERVER=${SOCK_CLIENT}

	atf_check -s exit:0 rump.ifconfig shmif0 create
	atf_check -s exit:0 rump.ifconfig shmif0 linkstr $BUS
	atf_check -s exit:0 rump.ifconfig shmif0 10.0.0.1/32
	atf_check -s exit:0 rump.ifconfig shmif0 up

	$DEBUG && rump.netstat -nr -f inet

	# Don't know a route to the gateway yet
	atf_check -s not-exit:0 -o match:'100.0% packet loss' \
	    -e match:'No route to host' rump.ping $PING_OPTS 192.168.0.1

	# Teach a route to the gateway
	atf_check -s exit:0 -o match:'add net 192.168.0.1: gateway shmif0' \
	    rump.route add -net 192.168.0.1/32 -link -cloning -iface shmif0
	atf_check -s exit:0 -o match:'add net default: gateway 192.168.0.1' \
	    rump.route add default -ifa 10.0.0.1 192.168.0.1

	$DEBUG && rump.netstat -nr -f inet

	# Be reachable to the gateway
	atf_check -s exit:0 -o ignore rump.ping $PING_OPTS 192.168.0.1

	unset RUMP_SERVER
}

non_subnet_gateway_cleanup()
{

	$DEBUG && shmif_dumpbus -p - $BUS 2>/dev/null | tcpdump -n -e -r -
	env RUMP_SERVER=$SOCK_CLIENT rump.halt
	env RUMP_SERVER=$SOCK_GW rump.halt
}

atf_test_case command_get cleanup
atf_test_case command_get6 cleanup
command_get_head()
{

	atf_set "descr" "tests of route get command"
	atf_set "require.progs" "rump_server"
}

command_get6_head()
{

	atf_set "descr" "tests of route get command (IPv6)"
	atf_set "require.progs" "rump_server"
}

setup_endpoint()
{
	local sock=${1}
	local addr=${2}
	local bus=${3}
	local mode=${4}
	local gw=${5}

	export RUMP_SERVER=${sock}
	atf_check -s exit:0 rump.ifconfig shmif0 create
	atf_check -s exit:0 rump.ifconfig shmif0 linkstr ${bus}
	if [ $mode = "ipv6" ]; then
		atf_check -s exit:0 rump.ifconfig shmif0 inet6 ${addr}
		atf_check -s exit:0 -o ignore rump.route add -inet6 default ${gw}
	else
		atf_check -s exit:0 rump.ifconfig shmif0 inet ${addr} netmask 0xffffff00
		atf_check -s exit:0 -o ignore rump.route add default ${gw}
	fi
	atf_check -s exit:0 rump.ifconfig shmif0 up

	if $DEBUG; then
		rump.ifconfig shmif0
		rump.netstat -nr
	fi
}

setup_forwarder()
{
	mode=${1}

	export RUMP_SERVER=$SOCKFWD
	atf_check -s exit:0 rump.ifconfig shmif0 create
	atf_check -s exit:0 rump.ifconfig shmif0 linkstr $BUS_SRCGW

	atf_check -s exit:0 rump.ifconfig shmif1 create
	atf_check -s exit:0 rump.ifconfig shmif1 linkstr $BUS_DSTGW

	if [ $mode = "ipv6" ]; then
		atf_check -s exit:0 rump.ifconfig shmif0 inet6 ${IP6SRCGW}
		atf_check -s exit:0 rump.ifconfig shmif1 inet6 ${IP6DSTGW}
	else
		atf_check -s exit:0 rump.ifconfig shmif0 inet ${IP4SRCGW} netmask 0xffffff00
		atf_check -s exit:0 rump.ifconfig shmif1 inet ${IP4DSTGW} netmask 0xffffff00
	fi

	atf_check -s exit:0 rump.ifconfig shmif0 up
	atf_check -s exit:0 rump.ifconfig shmif1 up

	if $DEBUG; then
		rump.netstat -nr
		if [ $mode = "ipv6" ]; then
			rump.sysctl net.inet6.ip6.forwarding
		else
			rump.sysctl net.inet.ip.forwarding
		fi
	fi
}

setup_forwarding()
{
	export RUMP_SERVER=$SOCKFWD
	atf_check -s exit:0 -o ignore rump.sysctl -w net.inet.ip.forwarding=1
}

setup_forwarding6()
{
	export RUMP_SERVER=$SOCKFWD
	atf_check -s exit:0 -o ignore rump.sysctl -w net.inet6.ip6.forwarding=1
}

setup()
{

	atf_check -s exit:0 rump_server $RUMP_LIBS $SOCKSRC
	atf_check -s exit:0 rump_server $RUMP_LIBS $SOCKFWD
	atf_check -s exit:0 rump_server $RUMP_LIBS $SOCKDST

	setup_endpoint $SOCKSRC $IP4SRC $BUS_SRCGW ipv4 $IP4SRCGW
	setup_endpoint $SOCKDST $IP4DST $BUS_DSTGW ipv4 $IP4DSTGW
	setup_forwarder ipv4
}

setup6()
{

	atf_check -s exit:0 rump_server $RUMP_LIBS_IPV6 $SOCKSRC
	atf_check -s exit:0 rump_server $RUMP_LIBS_IPV6 $SOCKFWD
	atf_check -s exit:0 rump_server $RUMP_LIBS_IPV6 $SOCKDST

	setup_endpoint $SOCKSRC $IP6SRC $BUS_SRCGW ipv6 $IP6SRCGW
	setup_endpoint $SOCKDST $IP6DST $BUS_DSTGW ipv6 $IP6DSTGW
	setup_forwarder ipv6
}

test_route_get()
{

	export RUMP_SERVER=$SOCKSRC
	$DEBUG && rump.netstat -nr -f inet
	$DEBUG && rump.arp -n -a

	# Make sure an ARP cache to the gateway doesn't exist
	rump.arp -d $IP4SRCGW

	# Local
	cat >./expect <<-EOF
   route to: 10.0.1.2
destination: 10.0.1.2
 local addr: 10.0.1.2
  interface: lo0
      flags: <UP,HOST,DONE,LLINFO,LOCAL>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get $IP4SRC > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output

	# Neighbor
	cat >./expect <<-EOF
   route to: 10.0.1.1
destination: 10.0.1.0
       mask: 255.255.255.0
 local addr: 10.0.1.2
  interface: shmif0
      flags: <UP,DONE,CONNECTED>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get $IP4SRCGW > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output

	# Remote host
	cat >./expect <<-EOF
   route to: 10.0.2.2
destination: default
       mask: default
    gateway: 10.0.1.1
 local addr: 10.0.1.2
  interface: shmif0
      flags: <UP,GATEWAY,DONE,STATIC>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get $IP4DST > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output

	# Create a ARP cache
	atf_check -s exit:0 -o ignore rump.ping -q -n -w $TIMEOUT -c 1 $IP4SRCGW

	# Neighbor with a cache (no different from w/o cache)
	cat >./expect <<-EOF
   route to: 10.0.1.1
destination: 10.0.1.0
       mask: 255.255.255.0
 local addr: 10.0.1.2
  interface: shmif0
      flags: <UP,DONE,CONNECTED>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get $IP4SRCGW > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output
}

test_route_get6()
{

	export RUMP_SERVER=$SOCKSRC
	$DEBUG && rump.netstat -nr -f inet
	$DEBUG && rump.ndp -n -a

	# Make sure an ARP cache to the gateway doesn't exist
	rump.ndp -d $IP6SRCGW

	# Local
	cat >./expect <<-EOF
   route to: fc00:0:0:1::2
destination: fc00:0:0:1::2
 local addr: fc00:0:0:1::2
  interface: lo0
      flags: <UP,HOST,DONE,LLINFO,LOCAL>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get -inet6 $IP6SRC > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output

	# Neighbor
	cat >./expect <<-EOF
   route to: fc00:0:0:1::1
destination: fc00:0:0:1::
       mask: ffff:ffff:ffff:ffff::
 local addr: fc00:0:0:1::2
  interface: shmif0
      flags: <UP,DONE,CONNECTED>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get -inet6 $IP6SRCGW > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output

	# Remote host
	cat >./expect <<-EOF
   route to: fc00:0:0:2::2
destination: ::
       mask: default
    gateway: fc00:0:0:1::1
 local addr: fc00:0:0:1::2
  interface: shmif0
      flags: <UP,GATEWAY,DONE,STATIC>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get -inet6 $IP6DST > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output

	# Create a NDP cache
	atf_check -s exit:0 -o ignore rump.ping6 -n -c 1 -X $TIMEOUT $IP6SRCGW

	# Neighbor with a cache (no different from w/o cache)
	cat >./expect <<-EOF
   route to: fc00:0:0:1::1
destination: fc00:0:0:1::
       mask: ffff:ffff:ffff:ffff::
 local addr: fc00:0:0:1::2
  interface: shmif0
      flags: <UP,DONE,CONNECTED>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0         0         0 
	EOF
	rump.route -n get -inet6 $IP6SRCGW > ./output
	$DEBUG && cat ./expect ./output
	atf_check -s exit:0 diff ./expect ./output
}

command_get_body()
{

	setup
	setup_forwarding
	test_route_get
}

command_get6_body()
{

	setup6
	setup_forwarding6
	test_route_get6
}

dump()
{

	env RUMP_SERVER=$SOCKSRC rump.netstat -nr
	env RUMP_SERVER=$SOCKFWD rump.netstat -nr
	env RUMP_SERVER=$SOCKDST rump.netstat -nr

	shmif_dumpbus -p - bus1 2>/dev/null| tcpdump -n -e -r -
	shmif_dumpbus -p - bus2 2>/dev/null| tcpdump -n -e -r -
}

cleanup()
{

	env RUMP_SERVER=$SOCKSRC rump.halt
	env RUMP_SERVER=$SOCKFWD rump.halt
	env RUMP_SERVER=$SOCKDST rump.halt
}

command_get_cleanup()
{
	$DEBUG && dump
	cleanup
}

command_get6_cleanup()
{
	dump
	cleanup
}

atf_init_test_cases()
{

	atf_add_test_case non_subnet_gateway
	atf_add_test_case command_get
	atf_add_test_case command_get6
}
