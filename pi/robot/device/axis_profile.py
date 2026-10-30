"""Emm位置模式曲线估计；参数依据厂家V1.0.5第57页。"""
import math

def acceleration(value=1):
    if type(value) is not int or not 0<=value<=255:
        raise ValueError('轴加速度档位必须为0～255整数；0为直接启动')
    return value

def travel_s(axis,distance_um,rpm,accel=1):
    accel=acceleration(accel)
    revolutions=abs(distance_um)/1000/(40.98 if axis=='lift' else 124.)
    if not revolutions:return 0.
    if not accel:return revolutions*60/rpm
    rate=1/((256-accel)*.00005)
    ramp_distance=rpm*rpm/(60*rate)
    return (2*math.sqrt(revolutions*60/rate) if revolutions<ramp_distance else
            revolutions*60/rpm+rpm/rate)
