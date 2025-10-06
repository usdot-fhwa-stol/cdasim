#!/usr/bin/env python3

"""
Utility to read net offset from SUMO net.xml files.
"""

import xml.etree.ElementTree as ET
import os
import logging

logger = logging.getLogger(__name__)

def read_net_offset_from_xml(net_xml_path: str) -> tuple[float, float]:
    """
    Read net offset from SUMO net.xml file.
    
    Args:
        net_xml_path: Path to the net.xml file
        
    Returns:
        Tuple of (x, y) offset values, or (0.0, 0.0) if not found or error
    """
    if not os.path.exists(net_xml_path):
        logger.warning(f"Net XML file does not exist: {net_xml_path}")
        return (0.0, 0.0)
    
    try:
        tree = ET.parse(net_xml_path)
        root = tree.getroot()
        
        # Look for <location> element with netOffset attribute
        for location in root.findall('location'):
            net_offset_attr = location.get('netOffset')
            if net_offset_attr:
                return parse_net_offset_string(net_offset_attr)
        
        logger.debug(f"No netOffset found in net.xml file: {net_xml_path}")
        return (0.0, 0.0)
        
    except Exception as e:
        logger.warning(f"Failed to parse net offset from {net_xml_path}: {e}")
        return (0.0, 0.0)

def parse_net_offset_string(net_offset_str: str) -> tuple[float, float]:
    """
    Parse net offset string in format "x,y" or "x,y,z".
    
    Args:
        net_offset_str: The net offset string
        
    Returns:
        Tuple of (x, y) values
    """
    try:
        # Handle format: "x,y" or "x,y,z"
        parts = net_offset_str.split(',')
        if len(parts) >= 2:
            x = float(parts[0].strip())
            y = float(parts[1].strip())
            logger.info(f"Parsed net offset: x={x}, y={y}")
            return (x, y)
    except (ValueError, IndexError) as e:
        logger.warning(f"Failed to parse net offset string '{net_offset_str}': {e}")
    
    return (0.0, 0.0)

def find_town04_net_xml() -> str:
    """
    Find Town04.net.xml file in common locations.
    
    Returns:
        Path to Town04.net.xml file, or empty string if not found
    """
    possible_paths = [
        # Relative to current directory
        "bundle/src/assembly/resources/scenarios/Town04/sumo/Town04.net.xml",
        "../bundle/src/assembly/resources/scenarios/Town04/sumo/Town04.net.xml",
        "../../bundle/src/assembly/resources/scenarios/Town04/sumo/Town04.net.xml",
        # Absolute paths (adjust as needed)
        "/path/to/Town04.net.xml",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            logger.info(f"Found Town04.net.xml at: {path}")
            return path
    
    logger.warning("Town04.net.xml not found in common locations")
    return ""

if __name__ == "__main__":
    # Test the function
    net_xml_path = find_town04_net_xml()
    if net_xml_path:
        offset = read_net_offset_from_xml(net_xml_path)
        print(f"Net offset: x={offset[0]}, y={offset[1]}")
    else:
        print("Town04.net.xml not found")
