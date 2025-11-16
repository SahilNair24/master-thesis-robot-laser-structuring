import csv

# Funktion zum Erstellen der jointtarget-Deklarationen
def create_jointtarget(index, joint_angles):
    angles_str = ",".join([f"{angle:.7f}" for angle in joint_angles])
    return f"CONST jointtarget JointTarget_{index}:=[[{angles_str}],[9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];"

# CSV-Datei lesen und Gelenkwinkel extrahieren
joint_targets = []
with open('/home/sahilsnair/Desktop/scripts/joint_positions.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        joint_targets.append([float(angle) for angle in row])

# Inhalt der TXT-Datei erstellen
with open('/home/sahilsnair/Desktop/scripts/laser_structuring_joint.txt', 'w') as f:
    f.write("MODULE Laserstrukturierung\n")
    f.write("PERS tooldata tLaserHead:=[TRUE,[[430,165,250],[0.5,0.5,-0.5,0.5]],[22,[-300,200,160],[1,0,0,0],0,0,0]];\n")

    for i, angles in enumerate(joint_targets, start=1):
        f.write(create_jointtarget(i, angles) + "\n")
    
    f.write("PROC main()\n")
    f.write("ConfL \\On;\n")
    f.write("ConfJ \\On;\n")
    
    for i in range(1, len(joint_targets) + 1):
        f.write(f"MoveAbsJ JointTarget_{i}, v100, fine, tLaserHead \\WObj:= wobj0;\n")
    
    f.write("WaitTime 2.0;\n")         
    f.write("ConfL \\Off;\n")
    f.write("ConfJ \\Off;\n")
    f.write("ENDPROC\n")
    f.write("ENDMODULE\n")
