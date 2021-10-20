# -*- coding: utf-8 -*-
u"""Common module （共通モジュール）

**Class Hierarchy Chart （クラス階層図）**

.. image:: i611_common.png
"""

import math
import threading
import socket
import time
import select
from contextlib import closing

def version_i611_common():
  u"""Private （非公開）"""
  return [0,3,3,0]

###======================================================================
###[例外クラス]
###======================================================================
#[Public] ############################
# 基底となる例外
class Robot_exception(Exception):
  u"""Base exception class （基底となる例外クラス）"""

  pass

#[Public] ############################
# 非常停止時に発生する例外
class Robot_emo(Robot_exception):
  u"""This exception occurs by emergency stop.（非常停止時に発生する例外[復帰はできません]）"""

  pass

#[Public] ############################
# 減速停止時に発生する例外
class Robot_stop(Robot_exception):
  u"""Exception which occurs by robot stop. （減速停止時に発生する例外）

  Note:
    This exception causes system error. This error can be reset.（i611Robotクラスのrelease_stopevent()によりリセットできます。）

  """

  pass

#[Public] ############################
# 力覚検知時に発生する例外
class Robot_forcesensor(Robot_exception):
  u"""Private (非公開)"""

  pass

#[Public] ############################
# エラー時に発生する例外
class Robot_error(Robot_exception):
  u"""Exception which occurs by program error. This error can be reset.（エラー時に発生する例外）"""

  pass

#[Public] ############################
# エラー時に発生する例外（復帰不能）
class Robot_fatalerror(Robot_exception):
  u"""Exception which occurs by fatal program error. （致命的エラー時に発生する例外（復帰はできません））"""

  pass

#[Public] ############################
# 電源OFF時に発生する例外
class Robot_poweroff(Robot_exception):
  u"""Exception which occurs when system shutdowns. （電源の遮断時に発生する例外（復帰はできません））"""

  pass


"""
======================================================================
内部処理用クラス
class _plsout_up():  ポーリング処理で使うことを想定。 一定回数呼ばれるまで"1"を返す
event(self, pls_num = 0):	pls_num回plsoutが呼ばれるまでplsout()の戻り値が"1"になる
plsout(self):	出力値

内部処理用クラス
class _pulse_up():  ポーリング処理で使うことを想定。 立ち上がりエッジを検出
pls(self, val):	valがFalseからTrueに変化すると"1"を返す

内部処理用クラス
class _pulse_down():  ポーリング処理で使うことを想定。 立ち下がりエッジを検出
pls(self, val):	valがTrueからFalseに変化すると"1"を返す

内部処理用クラス
class _pulse_change():  ポーリング処理で使うことを想定。 入力値の変化を検出
pls(self, val):	valが変化すると"1"を返す

内部処理用関数(客先非公開)
_bitflag(val, bit):	valを2進数表示して、bit桁目を読む
_args(val, key, typ, *arg1, **arg2):	可変個数引数、キーワード引数を解釈する
_chkparam(pram, **criteria):	パラメータの型と値の範囲をチェックする
_minv(_m):	逆行列
_mdotm(_m1, _m2):	行列×行列
_mdotv(_m, _v):	行列×ベクトル
_vdotv(_a, _b):	ベクトルの内積
_vcrossv(_a, _b):	ベクトルの外積
_vabs(v):	ベクトルの絶対値
_vnorm(v):	ベクトルの正規化
_vadd(v1, v2):  ベクトルの足し算
_vsub(v1, v2):  ベクトルの引き算
_mslice(_m, _r0, _r1, _c0, _c1):	行列のスライス
_matEye(_n):	単位行列
_matRx(_t):	X軸まわり回転のアフィン変換行列
_matRy(_t):	Y軸まわり回転のアフィン変換行列
_matRz(_t):	Z軸まわり回転のアフィン変換行列
_matEuler(_rz, _ry, _rx):	オイラー角からアフィン変換行列
_eulMatrix(_mat):	アフィン変換行列からオイラー角
_matShift(_x, _y, _z):	平行移動のアフィン変換行列
_matRotate(r, s, t):	ベクトル3つから回転行列をつくる
======================================================================
"""
class _plsout_up(object):
  def __init__(self):
    self.__pn = 0

  def event(self, pls_num = 0):
    if pls_num >= 0:
      self.__pn = pls_num
      return True
    else:
      return False

  def plsout(self):
    if self.__pn == 0:
      return 0
    else:
      self.__pn = self.__pn - 1
      return 1

class _pulse_up(object):
  def __init__(self):
    self.v1 = 1

  def pls(self, val):
    if (not self.v1) and val:
      self.v1 = val
      return 1
    else:
      self.v1 = val
      return 0

class _pulse_down(object):
  def __init__(self):
    self.v1 = 0

  def pls(self, val):
    if self.v1 and (not val):
      self.v1 = val
      return 1
    else:
      self.v1 = val
      return 0

class _pulse_change(object):
  def __init__(self):
    self.v1 = 0

  def pls(self, val):
    if bool(self.v1) != bool(val):
      self.v1 = val
      return 1
    else:
      self.v1 = val
      return 0

def _bitflag(val, bit):
  if type(bit) != int:
    raise Exception('Error invalid error')

  return bool((val & (1 << bit)))

def _args(val, key, typ, *arg1, **arg2):
  _v = dict(zip(key, val))
  _t = dict(zip(key, typ))
  i = 0

  def sub(v, k):
      if _t[k] == int:
        if type(v) != int:
          raise ValueError
        _v[k] = int(v)
      elif _t[k] == float:
        _v[k] = float(v)
      elif _t[k] == str:
        _v[k] = str(v)
      elif _t[k] == long:
        _v[k] = long(v)
      elif _t[k] == list:
        res = False
        for k in _t[k]:
          res = res or (type(v) == k)
      elif _t[k] == None:
          _v[k] = v
      else:
        if type(v) == _t[k]:
          _v[k] = v
        else:
          raise ValueError

  try:
    for a in arg1:
      if type(a) == list:
        for b in a:
          if i < len(key):
            sub(b, key[i])
          i += 1
      else:
         if i < len(key):
           sub(a, key[i])
         i += 1

    for k, v in arg2.items():
      _k = k.lower()
      if _k == 'line_speed':
        _k = 'lin_speed'
      if _k == 'joint_speed':
        _k = 'jnt_speed'
      sub(v, _k)

  except ValueError:
    return [False, 1]
  except KeyError:
    return [False, 3]
  except Exception as e:
    print 'i611_common, args exception'
    print e.message
    return [False, 99]
  else:
    return [True] + [_v[j] for j in key]

def _chkparam(pram, **criteria):
  if "p_type" in criteria:
    t = criteria["p_type"]
    res = False
    if isinstance(t,list):
      for k in t:
        res = res or isinstance(pram,k)

      #    elif isinstance(t,types.type)
    elif isinstance(t, type):
      res = (type(pram) == criteria["p_type"])

    if res == False:
      return [False, 1]

  if isinstance(pram,int) or isinstance(pram,float):
    if "min" in criteria:
      if pram < criteria["min"]:
        return [False, 2]

    if "max" in criteria:
      if pram > criteria["max"]:
        return [False, 2]

  return [True]

def _minv(_m):
  u"""逆行列"""
  n = len(_m) #行列の次元
  for i in range(n): #行列チェック
    if not len(_m[i]) == n:
      raise Exception("matrix error")
  m = [[float(_m[i][j]) for j in range(n)] + [1.0 if j == i else 0.0 for j in range(n)] \
    for i in range(n)]
  #ピボット選択
  for i in range(n):
    t = [abs(m[k][i]) for k in range(i, n)]
    m[i], m[t.index(max(t)) + i] = m[t.index(max(t)) + i], m[i]
    m[i] = [m[i][k]/m[i][i] for k in range(2*n)]
    for j in range(i+1,n):
      m[j] = [m[j][k] - m[i][k]*m[j][i] for k in range(2*n)]
  for i in reversed(range(n)):
    for j in range(0, i):
      m[j] = [m[j][k] - m[i][k]*m[j][i] for k in range(2*n)]
  return _mslice(m, 0, n, n, 2*n)

def _mdotm(_m1, _m2):
  u"""_mdotm(_m1, _m2)  :  行列のかけざん"""
  return [[_vdotv(_m1[r], [_m2[i][c] for i in range(len(_m2))])
    for c in range(len(_m2[0]))] for r in range(len(_m1))]

def _mdotv(_m, _v):
  u"""_mdotv(_m, _v)  :  行列とベクトルのかけざん"""
  return [_vdotv(_m[r], _v) for r in range(len(_m))]

def _vdotv(_a, _b):
  u"""_vdotv(_a, _b)  :  ベクトルの内積"""
  k = 0.0
  for i in range(len(_a)):
    k += _a[i] * _b[i]
  return k

def _vcrossv(_a, _b):
  u"""_vcrossv(_a, _b)  :  ベクトルの外積 """
  return [_a[1] * _b[2] - _a[2] * _b[1],
          _a[2] * _b[0] - _a[0] * _b[2],
          _a[0] * _b[1] - _a[1] * _b[0]]

def _vabs(v):
  u"""_vabs(v)  :  ベクトルの大きさ """
  return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def _vnorm(v):
  u"""_vnorm(v)  :  ベクトル正規化 """
  a = _vabs(v)
  return [v[0]/a, v[1]/a, v[2]/a]

def _vadd(v1, v2):
  u"""_vadd(v1, v2)  :  ベクトル加算 v1+v2 """
  return [v1[k] + v2[k] for k in range(len(v1))]

def _vsub(v1, v2):
  u"""_vsub(v1, v2)  :  ベクトルひきざん v1-v2 """
  return [v1[k] - v2[k] for k in range(len(v1))]

def _mslice(_m, _r0, _r1, _c0, _c1):
  u"""_mslice(_m, _r0, _r1, _c0, _c1)  :  行列のスライス"""
  return [k[_c0:_c1] for k in _m[_r0:_r1]]

def _matEye(_n):
  u"""_mEye(_n)  :  単位行列 """
  return [[1.0 if i == j else 0.0 for j in range(_n)] for i in range(_n)]

def _matRx(_t):
  u"""_matRx(_t)  :  x軸回り回転　変換行列"""
  c = math.cos(math.radians(_t))
  s = math.sin(math.radians(_t))
  return [
    [ 1., 0., 0., 0.],
    [ 0.,  c, -s, 0.],
    [ 0.,  s,  c, 0.],
    [ 0., 0., 0., 1.]
    ]

def _matRy(_t):
  u"""_matRy(_t)  :  y軸回り回転　変換行列"""
  c = math.cos(math.radians(_t))
  s = math.sin(math.radians(_t))
  return [
    [  c, 0.,  s, 0.],
    [ 0., 1., 0., 0.],
    [ -s, 0.,  c, 0.],
    [ 0., 0., 0., 1.]
    ]

def _matRz(_t):
  u"""_matRz(_t)  :  z軸回り回転　変換行列"""
  c = math.cos(math.radians(_t))
  s = math.sin(math.radians(_t))
  return [
    [ c, -s, 0., 0.],
    [ s,  c, 0., 0.],
    [0., 0., 1., 0.],
    [0., 0., 0., 1.]
    ]

def _matEuler(rz, ry, rx):
  cz = math.cos(math.radians(rz))
  sz = math.sin(math.radians(rz))
  cy = math.cos(math.radians(ry))
  sy = math.sin(math.radians(ry))
  cx = math.cos(math.radians(rx))
  sx = math.sin(math.radians(rx))

  return [[cy*cz, sx*sy*cz-cx*sz, sx*sz+cx*sy*cz, 0.],
          [cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz, 0.],
          [-sy  , sx*cy         , cx*cy         , 0.],
          [0., 0., 0., 1.]]

def _eulMatrix(m):
  M_PI = 3.1415926535897
  M_PI_2 = M_PI / 2.

  ry = math.asin(-m[2][0])
  if math.fabs(ry - M_PI_2) < 0.0000001:
    rz = math.atan2(m[1][1], m[0][1]) - M_PI_2
    ry = M_PI_2
    rx = 0.0
  elif math.fabs(ry + M_PI_2) < 0.0000001:
    rz = math.atan2(m[1][1], m[0][1]) - M_PI_2
    ry = -M_PI_2
    rx = 0.0
  else:
    rz = math.atan2(m[1][0], m[0][0])
    rx = math.atan2(m[2][1], m[2][2])

  return [math.degrees(rz), math.degrees(ry), math.degrees(rx)]


def _matShift(_x, _y, _z):
  return [
    [1., 0., 0., _x],
    [0., 1., 0., _y],
    [0., 0., 1., _z],
    [0., 0., 0., 1.]
    ]

def _matRotate(r, s, t):
  return [
    [r[0],s[0],t[0],0 ],
    [r[1],s[1],t[1],0 ],
    [r[2],s[2],t[2],0 ],
    [0   ,0   ,0   ,1 ]
    ]
"""======================================================================
[内部用クラス]
  _SockClient() : ソケット通信クライアント  本クラスを継承して使うことを想定

■ メソッド(関数)
connect(self, host, port, buff=256, wait=0.02):	通信開始
__comm(self):	通信機能本体
run(self):	スレッド本体
stop(self):	スレッド停止
setRecvData(self, data):	受信データを与える
getRecvData(self):	受信データの取得
setSendData(self, data):	送信データを与える
getSendData(self):	送信データを取得
======================================================================"""
class _SockClient(threading.Thread):
  def __init__(self):
    super(_SockClient, self).__init__()
    self.__flgStop = threading.Event()
    self.__lock_senddata = threading.Lock()
    self.__lock_recvdata = threading.Lock()
    self.__senddata = 'None'
    self.__recvdata = 'None'
    self.retry = 0
    self.setDaemon(True)
    self.host = None
    self.port = None
    self.buff = None
    self.wait = None
    self.sock = None

  def connect(self, host, port, buff=256, wait=0.02):
    self.host = host
    self.port = port
    self.buff = buff
    self.wait = wait
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.start()

  def __comm(self):
    try:
      with closing(self.sock):
        self.sock.connect((self.host, self.port))
        while True:
          time.sleep(0)
          if self.__flgStop.is_set():
            return False
          self.sock.send(self.getSendData())
          self.setRecvData(self.sock.recv(self.buff))
          time.sleep(self.wait)
    except Exception:
      return True
    return False

  def run(self):
    if self.retry == 0:
      self.__comm()
    else:
      while self.__comm():
        time.sleep(self.retry)

  def stop(self):
    self.__flgStop.set()

  def setRecvData(self, data):
    with self.__lock_recvdata:
      self.__recvdata = data

  def getRecvData(self):
    with self.__lock_recvdata:
      if self.__recvdata:
        data = self.__recvdata
        self.__recvdata = ''
        return data
      else:
        return 'None'

  def setSendData(self, data):
    with self.__lock_senddata:
      self.__senddata = data

  def getSendData(self):
    with self.__lock_senddata:
      if self.__senddata:
        data = self.__senddata
        self.__senddata = ''
        return data
      else:
        return 'None'

"""======================================================================
[内部用クラス]
  _SockServer(parent) : ソケット通信サーバー  本クラスを継承して使うことを想定

■ メソッド(関数)
connect(self, host, port, listen=1, buff=256, wait=0.001):	通信開始
__comm(self):	通信機能本体
run(self):	スレッド本体
stop(self):	スレッド停止
setRecvData(self, data):	受信データを与える
getRecvData(self):	受信データの取得
setSendData(self, data):	送信データを与える
getSendData(self):	送信データを取得
======================================================================"""
class _SockServer(threading.Thread):
  def __init__(self):
    super(_SockServer, self).__init__()
    self.__flgStop = threading.Event()
    self.__lock_senddata = threading.Lock()
    self.__lock_recvdata = threading.Lock()
    self.__senddata = 'None'
    self.__recvdata = 'None'
    self.retry = 10
    self.setDaemon(True)
    self.host = None
    self.port = None
    self.listen = None
    self.buff = None
    self.wait = None
    self.sock = None
    self.fds = None

  def connect(self, host, port, listen=1, buff=256, wait=0.001):
    self.host = host
    self.port = port
    self.listen = listen
    self.buff = buff
    self.wait = wait
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.start()

  def __comm(self):
    try:
      with closing(self.sock):
        self.fds = set([self.sock])
        self.sock.bind((self.host, self.port))
        self.sock.listen(self.listen)
        while True:
          time.sleep(0)
          if self.__flgStop.is_set():
            return False
          rlist, _, _ = select.select(self.fds, [], [])
          for sock in rlist:
            if sock is self.sock:
              conn, _ = self.sock.accept()
              self.fds.add(conn)
            else:
              msg = sock.recv(self.buff)
              if len(msg) == 0:
                sock.close()
                self.fds.remove(sock)
              else:
                self.setRecvData(msg)
                sock.send(self.getSendData())
          time.sleep(self.wait)
    except Exception as err_str:
      print 'Error comm:',err_str
      return True

  def run(self):
    if self.retry == 0:
      self.__comm()
    else:
      while self.__comm():
        time.sleep(self.retry)
        #self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  def stop(self):
    self.__flgStop.set()

  def setRecvData(self, data):
    self.__recvdata = data

  def getRecvData(self):
    if self.__recvdata:
      data = self.__recvdata
      self.__recvdata = ''
      return data
    else:
      return 'None'

  def setSendData(self, data):
    with self.__lock_senddata:
      self.__senddata = data

  def getSendData(self):
    with self.__lock_senddata:
      if self.__senddata:
        data = self.__senddata
        self.__senddata = ''
        return data
      else:
        return 'None'

#[EOF]
