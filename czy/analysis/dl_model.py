# -*- coding: utf-8 -*-
"""从 OSS 签名 URL 下载 checkpoint（URL 由 model_3600.b64 传入）"""
import base64
import sys
import urllib.request

with open(sys.argv[1]) as f:
    url = base64.urlsafe_b64decode(f.read().strip()).decode()
dst = sys.argv[2]
print("downloading ->", dst)
urllib.request.urlretrieve(url, dst)
import os
print("DL_OK size=", os.path.getsize(dst))
