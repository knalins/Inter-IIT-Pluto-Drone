#!/usr/bin/expect

set ip "192.168.4.1"

spawn "/bin/bash"
send "telnet $ip\r"
expect "'^]'."
send "+++AT MODE 3\r"
expect "#"
sleep 0.5

send "+++AT STA HawkAI blinkna123\r"
expect "#"

send -- "^]\r"
expect "telnet>"
send  "quit\r"
expect eof
