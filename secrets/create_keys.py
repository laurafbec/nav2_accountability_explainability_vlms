#!/usr/bin/python3
from pathlib import Path
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.exceptions import InvalidSignature

password = b"123456"

key_size = 2048  # Should be at least 2048

private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=key_size,
)

public_key = private_key.public_key()

key_pem_bytes = private_key.private_bytes(
   encoding=serialization.Encoding.PEM,  # PEM Format is specified
   format=serialization.PrivateFormat.PKCS8,
   encryption_algorithm=serialization.BestAvailableEncryption(password),
)

public_pem_bytes = public_key.public_bytes(
   encoding=serialization.Encoding.PEM,
   format=serialization.PublicFormat.SubjectPublicKeyInfo,
)

# pem files creation
key_pem_path = Path("key.pem")
key_pem_path.write_bytes(key_pem_bytes)

warning_message = "\n\n     TRUNCATED CONTENT TO REMIND THIS SHOULD NOT BE SHARED\n"

content = key_pem_path.read_text()
content = content[:232] + warning_message + content[1597:]

print(content)

public_pem_path = Path("public.pem")
public_pem_path.write_bytes(public_pem_bytes);

public_key_content = public_pem_path.read_text()
print(public_key_content)
