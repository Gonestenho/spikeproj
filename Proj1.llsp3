#Import những chức năng cần dùng
from hub import port, motion_sensor, button
import runloop,color_sensor, motor_pair, time, sys, color 

#Kết nối động cơ và cảm biến
def ket_noi():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)   #Cặp motor cổng A,B
    color_sensor.color(port.E)                           #Cảm biến màu cổng E

def yaw_90():
    return (motion_sensor.tilt_angles()[0] * -0.1) >= 90    #khi chỉ số yaw >=90 thì trả về true

def yaw_neg_90():
    return (motion_sensor.tilt_angles()[0] * -0.1) <= -90   #khi chỉ số yaw <= -90 thì trả về true

#tính PD
def PD ():
    lastError = 0  #Lỗi trước
    error = 75 - color_sensor.reflection(port.E) #Lỗi hiện tại
    P = error * 0.5  #Tính P
    derivative = error - lastError 
    lastError = error  #cập nhật lỗi trước bằng lỗi hiện tại
    D = derivative * 1 # Tính D
    correction = min(100, max(-100, int(P + D)))  # Giá trị sửa lỗi chỉ được dao động từ -100 -> 100
    return correction  #Gọi hàm thì sẽ trả về giá trị correction


#v là vận tốc vòng/s và s là quãng đường cm

#Đang tìm cách dò line bên phải, trái theo cm khác do cách này rủi ro cao và sai số do yếu tố xung quanh
#dò line bên phải theo cm
def fl_phai_cm(v, s):
    t= 19000*s/v  
    bat_dau= time.ticks_ms() 
    while time.ticks_ms() - bat_dau <= t:
        motor_pair.move_tank(motor_pair.PAIR_1, v - PD(), v + PD())

#dò line bên trái theo cm  
def fl_trai_cm(v, s):
    t= 19000*s/v
    bat_dau= time.ticks_ms()
    while time.ticks_ms() - bat_dau <= t:
        motor_pair.move_tank(motor_pair.PAIR_1, v - PD(), v + PD())


#dò line bên trái có dừng vạch đen
def fl_trai_vd(v):
    batdau= time.ticks_ms() #Đặt mốc thời gian đầu tiên bằng đồng hồ hiện tại
    stop= False 
    while color_sensor.reflection(port.E) >= 59 or stop == False : #Nếu xe chưa thấy vạch đen hoặc thực hiện chưa đủ 1s thì vẫn thực hiện tiếp
        if  time.ticks_ms()- batdau  > 1000 :  #Lệnh này giúp cho xe ổn định trong vòng 1s đầu
            stop= True
        motor_pair.move_tank(motor_pair.PAIR_1, v - PD(), v + PD())
    

#dò line bên phải có dừng vạch đen
def fl_phai_vd(v):
    batdau= time.ticks_ms()
    stop= False
    while color_sensor.reflection(port.E) >= 59 or stop == False :
        if time.ticks_ms()- batdau> 1000 :
            stop= True
        motor_pair.move_tank(motor_pair.PAIR_1, v + PD(), v - PD())


async def main():
    ket_noi() # Gọi hàm để kết nối với các port
    
    while not button.pressed(button.RIGHT): #chờ nhấn nút phải
        pass

    #fl đến vạch đen đầu tiên    
    fl_phai_vd(540)
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 100, 360, 360) #đi qua vạch đen
    motor_pair.stop(motor_pair.PAIR_1)

    #quay sang phải 90 độ
    motion_sensor.reset_yaw(0)   #reset yaw về giá trị 0
    await runloop.until(motion_sensor.stable) #reset cho đến khi ổn định (đoạn này dễ bị delay do phải đợi xe ổn định)
    motor_pair.move(motor_pair.PAIR_1, 100, velocity=200) #xoay tại chỗ bên phải với  vận tốc 200 vòng/s
    await runloop.until(yaw_90)  #lặp lại cho đến khi chỉ số yaw tăng lên đủ 90 độ, khi đủ thì hàm yaw_90 sẽ trả về true và kết thúc việc lặp lại
    motor_pair.stop(motor_pair.PAIR_1) # dừng động cơ để kết thúc việc xoay
    
    #đi tới 300 độ với tốc độ 360 độ/s
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 300, 360, 360)
    motor_pair.stop(motor_pair.PAIR_1)
    

    #xoay 180 độ
    motion_sensor.reset_yaw(900) 
    #Do yaw khi xoay theo chiều kim đồng hồ thì chỉ số từ 0 tăng lên 180 rồi đổi sang -180 tăng về lại 0
    #Nên ta sẽ reset yaw về -90 và cho quay đến 90 độ => tổng cộng 180 độ
    #900 vì chỉ số yaw lấy đơn vị là đề_xi độ (10 đề_xi độ = 1 độ) và luôn mang dấu âm (yaw = 900 <=> -90 độ)
    await runloop.until(motion_sensor.stable)
    motor_pair.move(motor_pair.PAIR_1, 100, velocity=200)
    await runloop.until(yaw_90)
    motor_pair.stop(motor_pair.PAIR_1)

    #đi tới 350 độ
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 350, 360, 360)
    motor_pair.stop(motor_pair.PAIR_1)
    
    #quay phải 90 độ
    motion_sensor.reset_yaw(0)
    await runloop.until(motion_sensor.stable)
    motor_pair.move(motor_pair.PAIR_1, 100, velocity=200)
    await runloop.until(yaw_90)
    motor_pair.stop(motor_pair.PAIR_1)

    #fl phải dừng vạch đen
    fl_trai_vd(540)
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 100, 360, 360)
    motor_pair.stop(motor_pair.PAIR_1)

    #quay trái 90 độ
    motion_sensor.reset_yaw(0)
    await runloop.until(motion_sensor.stable)
    motor_pair.move(motor_pair.PAIR_1, -100, velocity=200)
    await runloop.until(yaw_neg_90)
    motor_pair.stop(motor_pair.PAIR_1)

    fl_phai_vd(540)
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 650, 540, 540)

    motor_pair.stop(motor_pair.PAIR_1)

    #quay trái 90 độ
    motion_sensor.reset_yaw(0)
    await runloop.until(motion_sensor.stable)
    motor_pair.move(motor_pair.PAIR_1, -100, velocity=200)
    await runloop.until(yaw_neg_90)
    motor_pair.stop(motor_pair.PAIR_1)

    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 600, 540, 540)

    for i in range(5):    #fl phải dừng vạch đen 5 lần
        fl_phai_vd(360)
    
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 200, 360, 360)
    motor_pair.stop(motor_pair.PAIR_1)

    #quay phải 90 độ
    motion_sensor.reset_yaw(0)
    await runloop.until(motion_sensor.stable)
    motor_pair.move(motor_pair.PAIR_1, 100, velocity=200)
    await runloop.until(yaw_90)
    motor_pair.stop(motor_pair.PAIR_1)

    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 250, 360, 360)

runloop.run(main())
