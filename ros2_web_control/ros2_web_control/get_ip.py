import netifaces

def get_local_ip():
    for interface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(interface).get(netifaces.AF_INET)
        if addrs:
            for addr in addrs:
                ip = addr['addr']
                if not ip.startswith("127."):  
                    return ip
    return "127.0.0.1"  # Fallback