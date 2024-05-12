
import os
import rclpy
from rclpy.node import Node
from pathlib import Path

import cryptography
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization

class BagDecipherNode(Node):

    def __init__(self):

        # Initialize the node
        super().__init__('bag_decipher_node')

        # Get parameters values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ciphered_bag_dir', rclpy.Parameter.Type.STRING),
                ('ciphered_file_path', rclpy.Parameter.Type.STRING),
                ('private_key_path', rclpy.Parameter.Type.STRING),
                ('keystore_password', rclpy.Parameter.Type.STRING),
                ('encrypted_password', rclpy.Parameter.Type.STRING)

        ])

        ciphered_bag_dir = self.get_parameter('ciphered_bag_dir').get_parameter_value().string_value
        ciphered_file_path = self.get_parameter('ciphered_file_path').get_parameter_value().string_value
        private_key_path = self.get_parameter('private_key_path').get_parameter_value().string_value
        keystore_password = self.get_parameter('keystore_password').get_parameter_value().string_value
        encrypted_password = self.get_parameter('encrypted_password').get_parameter_value().string_value

        # Check if parameters are None, if so, use default values
        if ciphered_bag_dir is None:
            ciphered_bag_dir = os.path.join(os.getcwd(), "log_output")

        if ciphered_file_path is None:
            self.ciphered_file_path = "raw_data.txt"
        else:
            self.ciphered_file_path = ciphered_file_path
        
        # Get Encrypted Rosbag path URI
        self.uri = os.path.join(ciphered_bag_dir, ciphered_file_path)

        ## Asymetric decryption of the key
        private_pem_bytes = Path(private_key_path).read_bytes()
        keystore_password = keystore_password.encode('utf-8')

        try:
            private_key_from_pem = serialization.load_pem_private_key(
                private_pem_bytes,
                password=keystore_password,
            )
            self.get_logger().info("Private key correctly loaded")
        except ValueError:
            self.get_logger().info("Incorrect password for keystore")

        self.decrypted_password = self.decrypt(bytes.fromhex(encrypted_password),private_key_from_pem)
        

    # Decipher a Rosbag file
    def decipher_rosbag(self):
        # Open the txt file for writing messages
        log_file = self.uri
        password = self.decrypted_password

        with open(log_file, 'r') as logfile:
            for encrypted_message in logfile:
                nonce, ciphered_message = encrypted_message.split(":")
                nonce_bytes = bytes.fromhex(nonce)
                ciphered_message_bytes = bytes.fromhex(ciphered_message)

                print(self.verify(password, nonce_bytes, ciphered_message_bytes))

    # Verify if the message, the nonce or the key has been altered
    def verify(self,password, nonce, message):
        try:
            decrypted_message = AESGCM(password).decrypt(nonce, message, None)
            decrypted_message_str = decrypted_message.decode()

            return f"Decrypted Message: {decrypted_message_str}"
        except cryptography.exceptions.InvalidTag:
            return "Verification Failed - Either the message has been altered or the nonce or key are incorrect"
        
    def decrypt(self,message_encrypted, private_key):
        try:
            message_decrypted = private_key.decrypt(
            message_encrypted,
                padding.OAEP(
                    mgf=padding.MGF1(algorithm=hashes.SHA256()),
                    algorithm=hashes.SHA256(),
                    label=None
                )
            )
            return message_decrypted
        except ValueError:
            return "Failed to Decrypt"

   
def main(args=None):
    rclpy.init(args=args)
    sbr = BagDecipherNode()
    sbr.decipher_rosbag()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()