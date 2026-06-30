import socket
import struct
import sys

UDP_IP = "172.31.0.1"
UDP_PORT = 8888

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"--- AUV UDP Command Sender ---")
print(f"Target: {UDP_IP}:{UDP_PORT}")
print("Enter 4 comma-separated values for [Surge, Sway, Heave, Yaw] speeds.")
print("Example: 1.5, 0.0, -0.5, 0.2")
print("Type 'q' or 'exit' to quit.\n")

while True:
    try:
        user_input = input(">> ")
        
        if user_input.lower() in ['q', 'quit', 'exit']:
            print("Exiting...")
            break
            
        # Parse the input
        str_values = user_input.split(',')
        if len(str_values) != 4:
            print(f"Error: Expected exactly 4 values, but got {len(str_values)}. Please try again.")
            continue
            
        values = [float(x.strip()) for x in str_values]
        
        # Pack the 4 floats into a byte array representing 4 double-precision floats (8 bytes each)
        # '>' means Big-Endian (which matches your Simulink UDP block settings)
        # '4d' means 4 doubles
        data = struct.pack('>4d', values[0], values[1], values[2], values[3])
        
        # Send the byte array
        sock.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Sent: {values}")
        
    except ValueError:
        print("Error: Invalid input. Please ensure you are entering numbers separated by commas.")
    except KeyboardInterrupt:
        print("\nExiting...")
        break
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

sock.close()
