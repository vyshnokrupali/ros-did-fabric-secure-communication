import requests
import sys

FABRIC_API = "http://fabric-gateway:4000/verify"

def verify_device(did):
    try:
        response = requests.post(FABRIC_API, json={"did": did})
        return response.json().get("valid", False)
    except Exception as e:
        print("Identity verification failed:", e)
        return False

if __name__ == "__main__":
    did = sys.argv[1]
    if verify_device(did):
        print("DEVICE VERIFIED")
        sys.exit(0)
    else:
        print("DEVICE NOT VERIFIED")
        sys.exit(1)
