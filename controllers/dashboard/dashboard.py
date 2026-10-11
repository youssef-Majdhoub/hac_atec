import socket
from controller import Supervisor

UDP_ADDR = ("127.0.0.1", 9000)

supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())

receiver = supervisor.getDevice("telemetry")
receiver.enable(step)

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sent = 0

print("Dashboard bridge started, forwarding telemetry to", UDP_ADDR)

while supervisor.step(step) != -1:
    while receiver.getQueueLength() > 0:
        try:
            udp.sendto(receiver.getString().encode("utf-8"), UDP_ADDR)
            sent += 1
            if sent == 1:
                print("Dashboard bridge: first packet forwarded")
        except Exception as error:
            print("Dashboard bridge error:", error)
        finally:
            receiver.nextPacket()
