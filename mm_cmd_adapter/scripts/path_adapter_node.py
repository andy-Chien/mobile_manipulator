#!/usr/bin/env python3

import rclpy
from threading import Thread 
from mm_cmd_adapter import PathAdapter

def main(args=None):
    rclpy.init(args=args)
    node_name = 'path_adapter'
    path_adapter = PathAdapter(node_name)
    rclpy.spin(path_adapter)

if __name__ == '__main__':
    main()