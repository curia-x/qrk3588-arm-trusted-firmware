#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
"""Generate only the public SHA-256 DigestInfo; never copy private key bytes."""
import argparse
import hashlib
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--key", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    public = subprocess.run(
        ["openssl", "pkey", "-passin", "pass:", "-in", args.key, "-pubout", "-outform", "DER"],
        check=True, stdout=subprocess.PIPE).stdout
    metadata = subprocess.run(
        ["openssl", "rsa", "-pubin", "-inform", "DER", "-text", "-noout"],
        input=public, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).stdout
    if metadata.splitlines()[0] != b"Public-Key: (3072 bit)":
        parser.error("the root key must be RSA-3072")
    # ASN.1 SHA-256 AlgorithmIdentifier and 32-byte OCTET STRING.
    digest_info = bytes.fromhex("3031300d060960864801650304020105000420")
    digest_info += hashlib.sha256(public).digest()
    values = ", ".join(f"0x{value:02x}" for value in digest_info)
    content = ("/* Generated public trust anchor. SPDX-License-Identifier: BSD-3-Clause */\n"
               f"static const unsigned char qrk3588_rotpk[] = {{ {values} }};\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.exists() or args.output.read_text() != content:
        temporary = args.output.with_suffix(".tmp")
        temporary.write_text(content)
        temporary.replace(args.output)


if __name__ == "__main__":
    main()
