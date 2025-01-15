import re
import matplotlib.pyplot as plt


def parse_update_time(line):
    match = re.search(
        r'\d{2}:\d{2}:(\d+\.\d+).*?\[Update time\]:\s+(\d+\.\d+)', line)

    match = re.search(
        r'\d{2}:\d{2}:(\d+\.\d+).*?\[Update time\]:\s+(\d+\.\d+)', line)
    if match:
        time_str = match.group(1)
        minutes, seconds = map(float, time_str.split(':'))
        total_seconds = minutes * 60 + seconds
        time2 = match.group(2)
        print(total_seconds, time2)
        return total_seconds, time2

    # if match:
    #     time1, time2 = match.group(1), match.group(2)
    #     # print(time1, time2)
    #     return time1, time2
    return None, None


def extract_content(log_file_path, keyword):
    with open(log_file_path, 'r') as file:
        lines = file.readlines()
    t1_vals = []
    t2_vals = []
    cnt = []
    i = 0
    for line in lines:
        time1, time2 = parse_update_time(line)
        if time1 and time2:
            t1_vals.append(float(time1))
            t2_vals.append(float(time2))
            i = i + 1
            cnt.append(i)

    # plt.plot(t1_vals, t2_vals, marker='o')
    # plt.xlabel("timestamp")
    # plt.ylabel("time cusumed")
    # plt.show()

    # plt.plot(cnt, t1_vals, marker='o')
    # plt.xlabel("timestamp")
    # plt.ylabel("time cusumed")
    # plt.show()

    plt.plot(cnt, t2_vals, marker='o')
    plt.xlabel("timestamp")
    plt.ylabel("time cusumed")
    plt.show()


if __name__ == "__main__":
    # Update this path to your log file
    log_file_path = '/home/mint/ws_uav_setup/src/Coco-LIC-UWB/log/odometry_node.INFO'
    # Update this keyword to search for different keywords
    keyword = '[Update time]'
    content = extract_content(log_file_path, keyword)
