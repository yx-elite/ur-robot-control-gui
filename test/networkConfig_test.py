import subprocess
import re

def get_ipv4_addresses():
    ipv4_addresses = []

    try:
        # Run ipconfig command and capture its output
        ipconfig_output = subprocess.check_output(["ipconfig", "/all"], universal_newlines=True)

        # Use regular expression to find IPv4 addresses in the output
        ipv4_pattern = r"IPv4 Address(?:.*): (\d+\.\d+\.\d+\.\d+)"
        matches = re.findall(ipv4_pattern, ipconfig_output)

        # Add matched IPv4 addresses to the list
        for match in matches:
            ipv4_addresses.append(match)
    except subprocess.CalledProcessError:
        print("Error running ipconfig command.")

    return ipv4_addresses

# Test the function
ipv4_addresses = get_ipv4_addresses()
print("IPv4 Addresses:", ipv4_addresses)