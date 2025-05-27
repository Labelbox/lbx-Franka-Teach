#!/usr/bin/env python3
"""
Simple VR Server - Receives binary VR data and converts to controller text format
"""

import socket
import struct
import time

def parse_binary_to_controller_format(data):
    """Convert binary VR data to controller text format"""
    try:
        if len(data) < 10:
            return None
            
        # For now, let's try to extract some basic info from the binary data
        # The exact format depends on what the VR app is sending
        
        # Try to interpret first few bytes as different data types
        hex_data = data.hex()
        
        # Mock controller state based on binary data analysis
        # We'll need to reverse engineer the actual format
        
        # Check if we can extract any meaningful values
        try:
            # Try interpreting as floats (little endian)
            if len(data) >= 4:
                float_val = struct.unpack('<f', data[:4])[0]
            else:
                float_val = 0.0
        except:
            float_val = 0.0
            
        # Create controller format string
        # Format: left;x:bool;y:bool;menu:bool;thumbstick:bool;index_trigger:float;hand_trigger:float;thumbstick_axes:x,y;position:x,y,z;rotation:w,x,y,z;|right;...
        
        controller_text = (
            f"left;"
            f"x:false;"
            f"y:false;"
            f"menu:false;"
            f"thumbstick:false;"
            f"index_trigger:0.0;"
            f"hand_trigger:0.0;"
            f"thumbstick_axes:0.0,0.0;"
            f"position:0.0,0.0,0.0;"
            f"rotation:1.0,0.0,0.0,0.0;"
            f"|"
            f"right;"
            f"a:false;"
            f"b:false;"
            f"menu:false;"
            f"thumbstick:false;"
            f"index_trigger:{float_val:.3f};"
            f"hand_trigger:0.0;"
            f"thumbstick_axes:0.0,0.0;"
            f"position:0.0,0.0,0.0;"
            f"rotation:1.0,0.0,0.0,0.0;"
        )
        
        return controller_text
        
    except Exception as e:
        print(f"Error parsing binary data: {e}")
        return None

def analyze_binary_structure(data):
    """Analyze binary data structure to understand the format"""
    analysis = {
        'length': len(data),
        'hex': data.hex(),
        'first_4_bytes_as_int': None,
        'first_4_bytes_as_float': None,
        'ascii_attempt': None
    }
    
    try:
        if len(data) >= 4:
            analysis['first_4_bytes_as_int'] = struct.unpack('<I', data[:4])[0]
            analysis['first_4_bytes_as_float'] = struct.unpack('<f', data[:4])[0]
    except:
        pass
        
    try:
        analysis['ascii_attempt'] = data.decode('ascii', errors='ignore')
    except:
        pass
        
    return analysis

def test_connection_alive(client_socket):
    """Test if the connection is still alive by trying to send data"""
    try:
        # Try to send a small probe
        client_socket.send(b"")
        return True
    except:
        return False

def start_vr_server():
    # Create TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to WiFi interface
    server.bind(('0.0.0.0', 5555))
    server.listen(1)
    
    print("VR Server with Binary-to-Text Converter")
    print("Listening on 0.0.0.0:5555")
    print("VR app should connect to: 192.168.1.54:5555")
    print("=" * 80)
    
    try:
        while True:
            # Accept connection
            client_socket, address = server.accept()
            print(f"✅ VR app connected from {address}")
            print("-" * 80)
            
            message_count = 0
            last_heartbeat = time.time()
            last_data_time = time.time()
            
            try:
                while True:
                    # Set shorter timeout for recv
                    client_socket.settimeout(0.5)
                    
                    try:
                        # Receive data
                        data = client_socket.recv(1024)
                        if not data:
                            print("❌ VR app disconnected (no data)")
                            break
                        
                        message_count += 1
                        last_data_time = time.time()
                        
                        print(f"\n[{message_count:04d}] === RECEIVED DATA ===")
                        
                        # Analyze binary structure
                        analysis = analyze_binary_structure(data)
                        print(f"Length: {analysis['length']} bytes")
                        print(f"Hex: {analysis['hex']}")
                        if analysis['first_4_bytes_as_int'] is not None:
                            print(f"First 4 bytes as int: {analysis['first_4_bytes_as_int']}")
                        if analysis['first_4_bytes_as_float'] is not None:
                            print(f"First 4 bytes as float: {analysis['first_4_bytes_as_float']:.6f}")
                        if analysis['ascii_attempt']:
                            print(f"ASCII attempt: '{analysis['ascii_attempt']}'")
                        
                        # Convert to controller format
                        controller_text = parse_binary_to_controller_format(data)
                        if controller_text:
                            print(f"\n📝 CONVERTED TO CONTROLLER FORMAT:")
                            print(f"{controller_text}")
                        else:
                            print("❌ Could not convert to controller format")
                        
                        # Send simple acknowledgment
                        client_socket.send(b"OK\n")
                        
                    except socket.timeout:
                        # Check if we haven't received data for too long
                        current_time = time.time()
                        time_since_last_data = current_time - last_data_time
                        
                        if time_since_last_data > 10:  # 10 seconds without data
                            print(f"⚠️  No data received for {time_since_last_data:.1f} seconds")
                            # Test if connection is still alive
                            if not test_connection_alive(client_socket):
                                print("❌ Connection appears to be dead, closing...")
                                break
                        
                        # Show heartbeat every 5 seconds
                        if current_time - last_heartbeat > 5:
                            print(f"💓 Heartbeat - {message_count} messages received, last data {time_since_last_data:.1f}s ago")
                            last_heartbeat = current_time
                    
            except Exception as e:
                print(f"❌ Error: {e}")
            finally:
                client_socket.close()
                print("\n" + "=" * 80)
                print("Connection closed, waiting for new connection...")
                print("=" * 80)
                
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        server.close()

if __name__ == "__main__":
    start_vr_server() 