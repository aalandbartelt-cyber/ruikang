# -*- coding: utf-8 -*-
import socket
import json

def main():
    udp_ip = "127.0.0.1"
    udp_port = 8080  # 对应 C++ 接收视觉数据的端口
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"🎯 伪造数据桥接器 (fake_data_bridge) 已启动！目标: {udp_ip}:{udp_port}")
    print("======================================================")
    print("指令格式说明 (空格隔开)：")
    print("  黑线偏移   -> 输入: o <数值>   (例: o 50, o -20)")
    print("  高优警示牌 -> 输入: w <类型>   (例: w ELEC, w RAD, w FIRE)")
    print("  二维码识别 -> 输入: a <ID>     (例: a 1, a 2)")
    print("  恢复静默   -> 输入: 0          (发送全默认安全状态)")
    print("  退出程序   -> 输入: q")
    print("======================================================")

    while True:
        choice = input("\n请输入指令: ").strip()
        
        if choice.lower() == 'q':
            break
            
        if not choice:
            continue

        # 每次发送前重置为默认安全状态 (防止上一次的警示牌一直残留)
        payload = {
            "line_offset": 0.0,
            "obstacle_distance": 9.99,  # 保留此项防止 C++ JSON 解析崩溃
            "platform_tag": 0,
            "warning_sign": "NONE",
            "aruco_detected": False
        }

        # 处理直接回车恢复静默的情况
        if choice == '0':
            pass
        else:
            # 切割输入的指令，例如把 "w ELEC" 切成 ['w', 'ELEC']
            parts = choice.split()
            cmd = parts[0].lower()

            try:
                # 1. 解析偏移量: o 50
                if cmd == 'o':
                    payload["line_offset"] = float(parts[1])
                    
                # 2. 解析警示牌: w ELEC
                elif cmd == 'w':
                    sign = parts[1].upper()
                    if sign == 'ELEC':
                        payload["warning_sign"] = "ELECTRIC"
                    elif sign == 'RAD':
                        payload["warning_sign"] = "RADIATION"
                    elif sign == 'FIRE':
                        payload["warning_sign"] = "FIRE"
                    else:
                        payload["warning_sign"] = sign # 也支持随便输个自定义字符串
                        
                # 3. 解析二维码: a 1
                elif cmd == 'a':
                    payload["aruco_detected"] = True
                    payload["platform_tag"] = int(parts[1])
                    
                else:
                    print("⚠️ 未知指令前缀，请使用 o, w, a 或 0")
                    continue
                    
            except (IndexError, ValueError):
                print("⚠️ 参数格式错误！请检查是否漏打空格或输错类型 (例如: o 50)")
                continue

        # 打包发送
        json_str = json.dumps(payload)
        sock.sendto(json_str.encode('utf-8'), (udp_ip, udp_port))
        print(f"📡 已发送 -> {json_str}")

if __name__ == "__main__":
    main()