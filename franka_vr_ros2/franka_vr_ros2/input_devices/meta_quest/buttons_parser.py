"""Button parsing utilities for Meta Quest controllers"""


def parse_buttons(text: str) -> dict:
    """
    Parse button state string from Meta Quest companion app
    
    Args:
        text: Button state string from the app
        
    Returns:
        Dictionary of button states and analog values
    """
    split_text = text.split(',')
    buttons = {}
    
    # Right hand buttons
    if 'R' in split_text:
        split_text.remove('R')  # Remove marker
        buttons.update({
            'A': False,
            'B': False,
            'RThU': False,      # Right thumb up from rest position
            'RJ': False,        # Right joystick pressed
            'RG': False,        # Right grip (boolean from SDK)
            'RTr': False        # Right trigger (boolean from SDK)
        })
        
    # Left hand buttons
    if 'L' in split_text:
        split_text.remove('L')  # Remove marker
        buttons.update({
            'X': False,
            'Y': False,
            'LThU': False,      # Left thumb up from rest position
            'LJ': False,        # Left joystick pressed
            'LG': False,        # Left grip (boolean from SDK)
            'LTr': False        # Left trigger (boolean from SDK)
        })
        
    # Process boolean buttons
    for key in list(buttons.keys()):
        if key in split_text:
            buttons[key] = True
            split_text.remove(key)
            
    # Process analog values
    # Format: "key value1 value2 ..."
    # Examples:
    # - 'rightJS'/'leftJS': (x, y) joystick position, both in range (-1.0, 1.0)
    # - 'rightGrip'/'leftGrip': float trigger value in range (0.0, 1.0)
    # - 'rightTrig'/'leftTrig': float trigger value in range (0.0, 1.0)
    for elem in split_text:
        split_elem = elem.split(' ')
        if len(split_elem) < 2:
            continue
            
        key = split_elem[0]
        values = [float(x) for x in split_elem[1:]]
        
        # Store as tuple if multiple values, otherwise as single float
        if len(values) == 1:
            buttons[key] = values[0]
        else:
            buttons[key] = tuple(values)
            
    return buttons 