def process_gcode(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    output_lines = []
    in_g0_sequence = False
    pending_g0_coords = []
    
    for line in lines:
        line = line.strip()
        
        # Skip empty lines
        if not line:
            continue
            
        # Handle G0 commands (rapid movement)
        if line.startswith('G0'):
            parts = line.split()
            has_xy = any(p.startswith(('X', 'Y')) for p in parts)
            has_z = any(p.startswith('Z') for p in parts)
            
            # Start G0 sequence if we have XY movement
            if not in_g0_sequence and has_xy:
                output_lines.append('M300')  # Laser off
                in_g0_sequence = True
            
            # Process XY coordinates if present
            if has_xy:               
                # Extract coordinates
                coords = []
                for part in parts:
                    if part.startswith('X'):
                        coords.append(part)
                    elif part.startswith('Y'):
                        coords.append(part)
                
                # Add coordinate line
                if coords:
                    output_lines.append(' '.join(coords))
            
            # For Z-only moves, do nothing (ignore)
            
        # Handle G1 commands (linear interpolation with laser)
        elif line.startswith('G1'):
            # End any G0 sequence
            if in_g0_sequence:
                output_lines.append('M301')  # Laser on
                in_g0_sequence = False
            
            # Extract and format coordinates (ignore Z)
            parts = line.split()
            coords = []
            for part in parts:
                if part.startswith('X') or part.startswith('Y'):
                    coords.append(part)
            
            # Add coordinate line(s) if XY present
            if coords:
                coord_line = ' '.join(coords)
                output_lines.append(coord_line)
                
        # Handle other commands
        else:
            # End any G0 sequence
            if in_g0_sequence:
                output_lines.append('M301')  # Laser on
                in_g0_sequence = False
            # output_lines.append(line)
    
    # Handle case where file ends with G0 commands
    if in_g0_sequence:
        output_lines.append('M300')  # Laser off
    
    # Write the processed output
    with open(output_file, 'w') as f:
        f.write('T500\n')
        f.write('\n'.join(output_lines))
        f.write('\nM300')

# Example usage:
process_gcode('laser2largepockets.gcode', 'laser2largepockets.mpf')