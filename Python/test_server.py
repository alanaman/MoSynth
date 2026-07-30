import zmq
import struct

def main():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")

    print("Server listening on port 5555...")

    while True:
        # Receive byte array from Unity
        message = socket.recv()
        
        # Calculate how many floats are in the byte array (4 bytes per float)
        num_floats = len(message) // 4
        
        # Unpack the bytes into floats (using little-endian format '<')
        floats = struct.unpack(f'<{num_floats}f', message)
        print(f"Received: {floats}")
        
        # Reply with the exact same byte array
        socket.send(message)

if __name__ == "__main__":
    main()