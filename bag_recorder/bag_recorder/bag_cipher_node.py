import rclpy
from rclpy.node import Node

import os
from pathlib import Path

import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import secrets
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives import serialization

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding

class BagCipherNode(Node):

    
    def __init__(self):

        # Initialize the node
        super().__init__('bag_cipher_node')

        # Get parameters values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rosbag_dir', rclpy.Parameter.Type.STRING),
                ('rosbag_file_path', rclpy.Parameter.Type.STRING),
                ('ciphered_bag_dir', rclpy.Parameter.Type.STRING),
                ('ciphered_file_path', rclpy.Parameter.Type.STRING),
                ('public_key_path', rclpy.Parameter.Type.STRING),
                ('truststore_password', rclpy.Parameter.Type.STRING)
        ])

        rosbag_dir = self.get_parameter('rosbag_dir').get_parameter_value().string_value
        rosbag_file_path = self.get_parameter('rosbag_file_path').get_parameter_value().string_value
        ciphered_bag_dir = self.get_parameter('ciphered_bag_dir').get_parameter_value().string_value
        ciphered_file_path = self.get_parameter('ciphered_file_path').get_parameter_value().string_value
        public_key_path = self.get_parameter('public_key_path').get_parameter_value().string_value
        truststore_password = self.get_parameter('truststore_password').get_parameter_value().string_value

        # Check if parameters are None, if so, use default values
        if rosbag_dir is None:
            rosbag_dir = os.path.join(os.getcwd(), "rosbag_output")

        if rosbag_file_path is None:
            self.get_logger().info("Rosbag file path is missing.")

        if ciphered_bag_dir is None:
            ciphered_bag_dir = os.path.join(os.getcwd(), "log_output")

        if ciphered_file_path is None:
            self.ciphered_file_path = "raw_data.txt"
        else:
            self.ciphered_file_path = ciphered_file_path
        
        # Get Rosbag path URI
        self.uri = os.path.join(rosbag_dir, rosbag_file_path)       

        # Create a directory to store the log file
        self.log_dir = ciphered_bag_dir
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)

        self.key = secrets.token_bytes(32) # 16, 24, or 32

        #self.get_logger().info("Key: %s" % self.key.hex())

        ## Asymetric encryption of the key
        public_pem_bytes = Path(public_key_path).read_bytes()
        truststore_password = truststore_password.encode('utf-8')


        try:
            public_key_from_pem = serialization.load_pem_public_key(public_pem_bytes)
            self.get_logger().info("Public key correctly loaded")
            encrypted_key = self.encrypt(self.key, public_key_from_pem)
            self.get_logger().info("Encrypted Key: %s" % encrypted_key.hex())
        except ValueError:
            self.get_logger().info("Incorrect password for truststore")

      

    # Read Rosbag file
    def read_rosbag(self):
        storage_options, converter_options = self.get_rosbag_options(self.uri, storage_id='sqlite3')

        # Reader object
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
       
        topic_types = reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        # Set filter for topic of string type
        storage_filter = rosbag2_py.StorageFilter(topics=['rosout','amcl_pose','navigate_to_pose/_action/status','plan','scan','behavior_tree_log','cmd_vel'])
        reader.set_filter(storage_filter)
      
        # Open the txt file for writing messages
        log_file = os.path.join(self.log_dir, self.ciphered_file_path)

        nonce = secrets.token_bytes(24)

        with open(log_file, 'w') as logfile:
            while reader.has_next():
                (topic, data, t) = reader.read_next()

                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

                message_str = str(msg)
                message_bytes = message_str.encode()

                ciphered_message = AESGCM(self.key).encrypt(nonce, message_bytes, None)                
                encrypted_message = f"{nonce.hex()}:{ciphered_message.hex()}"

                logfile.write(str(encrypted_message) + "\n")         
                

    # Get Rosbag options
    def get_rosbag_options(self, path, storage_id, serialization_format='cdr'):
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format=serialization_format, output_serialization_format=serialization_format)
        return storage_options, converter_options
    
    # Encryption function RSA
    def encrypt(self, message, public_key):
        return public_key.encrypt(
            message,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )
     
def main(args=None):
    rclpy.init(args=args)
    sbr = BagCipherNode()
    sbr.read_rosbag()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()