import csv
from scipy.spatial.transform import Rotation as R

# Funktion zum Erstellen der jointtarget-Deklarationen
def create_robtarget(index, position, quaternion_wxyz):
    pos_str = ",".join([f"{p:.7f}" for p in position])
    quat_str = ",".join([f"{q:.7f}" for q in quaternion_wxyz])
    return f"CONST robtarget Target_{index}:=[[{pos_str}], [{quat_str}], [0,0,0,0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];"

# CSV-Datei lesen und Gelenkwinkel extrahieren
rob_targets = []
with open('/home/sahilsnair/Desktop/laser-toolpath/scripts/fk_results.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        pos = [float(val) for val in row[:3]]
        euler_angles = [float(val) for val in row[3:6]]
        quat_xyzw = R.from_euler('xyz', euler_angles).as_quat()  # [x, y, z, w]
        quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        rob_targets.append((pos, quat_wxyz))

# Inhalt der TXT-Datei erstellen
with open('/home/sahilsnair/Desktop/laser-toolpath/scripts/laser_structuring_joint.txt', 'w') as f:
    f.write("MODULE Laserstrukturierung\n")
    f.write("PERS tooldata tLaserHead:=[TRUE,[[430,165,250],[0.5,0.5,-0.5,0.5]],[22,[-300,200,160],[1,0,0,0],0,0,0]];\n")

    for i, (pos, quat) in enumerate(rob_targets, start=1):
        f.write(create_robtarget(i, pos, quat) + "\n")
    
    f.write("PROC main()\n")
    f.write("ConfL \\On;\n")
    f.write("ConfJ \\On;\n")
    
    for i in range(1, len(rob_targets) + 1):
        f.write(f"MoveJ Target_{i}, v100, fine, tLaserHead \\WObj:= wobj0;\n")
    
    f.write("WaitTime 2.0;\n")         
    f.write("ConfL \\Off;\n")
    f.write("ConfJ \\Off;\n")
    f.write("ENDPROC\n")
    f.write("ENDMODULE\n")
