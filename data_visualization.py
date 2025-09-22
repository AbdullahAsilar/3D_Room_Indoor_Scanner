import serial
import numpy as np
import open3d as o3d
import math

# Define the angle sequence (in degrees) we are traveling clockwise 

angles_deg = [
    
    180.00, 168.75, 157.50, 146.25, 135.00, 123.75, 112.50, 101.25,
    90.00, 78.75, 67.50, 56.25, 45.00, 33.75, 22.50, 11.25,
    0.00, -11.25, -22.50, -33.75, -45.00, -56.25, -67.50, -78.75,
    -90.00, -101.25, -112.50, -123.75, -135.00, -146.25, -157.50, -168.75
]

z_values = [0, 1000,2000,3000,4000,5000,6000,7000,8000, 9000, 10000, 11000 ,12000, 13000, 14000,15000]
j = 0 # this is an index value for Z_values 
k = 0  # this is the index for angles 
a= 1
# Serial setup
s = serial.Serial('COM3', 115200, timeout=10)
print("Opening: " + s.name)
s.reset_output_buffer()
s.reset_input_buffer()

count = 96
processed = 0

# Open file for writing
with open("tof_radar.xyz", "w") as f: 
    # Signal MCU to start
    input("press enter")
    s.write('s'.encode())
    
    
    
    while (processed != count):

        
        try:
            print("current spot" , processed%32)
            raw_data = s.readline().decode().strip()  # E.g., "0, 16, 41720, 0, 1"
            print(f"Raw data (string): {raw_data}")  # Debug output

            # Split into parts using commas
            parts = raw_data.split(',')  # Now a list: ["0", " 16", " 41720", " 0", " 1"]
            print(f"Split parts: {parts}")  # Debug output

            # Extract distance (2nd value), remove spaces, convert to int
            distance = int(parts[1].strip())  # " 16" → "16" → 16
            print(f"Distance extracted: {distance}")  # Debug output

            # is there a need for this line here? 
            if len(parts) >= 2:
                distance = int(parts[1].strip())  # Get 2nd value as distance
            else:
                print(f"Invalid data format: {raw_data}")
                continue
            
            # Convert to Cartesian coordinates
            j = processed // len(angles_deg)  # z-index
            k = processed % len(angles_deg)   # angle index

            angle_rad = math.radians(angles_deg[k])
            x = math.cos(angle_rad) * distance
            y = math.sin(angle_rad) * distance
            z = z_values[j]
            

            # # if a != 0:
            # if (a %(len(angles_deg))) ==0 : 
            #     j += 1 # z value 
            #     k= -1 #angle 
            # a+=1    
            # k +=1 
            

            print(f"z value: {z}")
            
            
            # Write to file
            f.write(f"{x:.2f} {y:.2f} {z}\n")
            print(f"Processed: Angle={angles_deg[k]}°, Distance={distance} → ({x:.2f}, {y:.2f}, {z})")
            processed += 1; 
        except (ValueError, IndexError) as e:
            print(f"Error processing line: {e}")
            continue

# Read and visualize point cloud
print("\nReading point cloud...")
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")
if not pcd.points:
    print("Warning: Point cloud is empty! Check output file.")
else:
    print("Point cloud array:")
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd], window_name="Radar Scan")

   #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,96):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    # Connect lines within each yz slice

    for x in range(0, 96, 32):
        for i in range(32 - 1):
            lines.append([x + i, x + i + 1])
        lines.append([x + 32 - 1, x])  # Close loop

    # Inter-slice connections (vertical lines)
    for x in range(0, 64, 32):  # 64 = 96-32 (last slice has no next)
        for i in range(32):
            lines.append([x + i, x + i + 32])

   
    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
                                    
    

s.close()
print("Done.")

