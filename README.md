# mip
## ТЗ
5) обеспечить переход двухзвенного маятника в заданное декартово положение за счет вычисления необходимого положения джоинтов через p.calculateInverseKinematics в режиме POSITION_CONTROL
## Решение
Решение собственно банальное: 

Вместо куска кода с ручным счетом якобиана и вычисление через ошибку положения

```python
jac = np.array([
    [(-L1*np.cos(th1) - L2*np.cos(th1+th2)), -L2*np.cos(th1+th2)],
    [(L1*np.sin(th1) + L2*np.sin(th1+th2)), L2*np.sin(th1+th2)]
])

jac_inv = np.linalg.inv(jac)
...
vel_d = -100.0 * jac_inv @ (X-Xd_curr)
vel_d = vel_d.flatten()

p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=[1,3], targetVelocities=vel_d, controlMode=p.VELOCITY_CONTROL)
p.stepSimulation()
```
был заменен на кусок
```python
kin = p.calculateInverseKinematics(boxId, 4, [Xd_curr[0][0], 0, Xd_curr[1][0]])
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=[1,3], targetPositions=kin, controlMode=p.POSITION_CONTROL)
p.stepSimulation()
```
# Итог
Изменено только пару строк, собственно как и предполагалось. Измененный файл [two-link/pendulum.py](two-link/pendulum.py)
