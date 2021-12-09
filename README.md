# MPU6050 with Kalman Filter
Kalman Filter based Sensor Fusion Algorithm (For Raspberry Pi and Arduino Serial Communication)  


### - Problem of changing to Euelr Angle 

- 자이로센서

자이로센서로 측정한 각속도를 오일러 각도의 변화율로 바꿔서 적분하여 오일러 각도를 얻어낸다. 하지만 자이로의 측정값은 오차가 섞여 있기 때문에 적분하여 구한 자세에서도 오차가 포함되게 된다. 

```c++
void GyroXYZ(){
  getRawData();
  rateGyX = GyX/131.0; // p
  rateGyY = GyY/131.0; // q
  rateGyZ = GyZ/131.0; // r
} // Sensor.ino
```
측정된 각속도를 보면 X축 방향 각속도가 수평 상태에서도 -9.XX값을 보이며 오차가 섞여 있는 것을 확인할 수 있다. 따라서 적분하여 구한 자세에서도 오차가 발생하게 된다. 시간이 지속되게 되면 오차가 누적되어 값이 편향된다.

- 가속도센서
```c++
void EulerAccel(){
   phi = atan(AcY/AcZ);
   theta = atan(AcX/sqrt(pow(AcY,2) + pow(AcZ,2)));
   psi = 0;
   // not enough to use!!
} // topic_subscriber.cpp
```
가속도센서는 일반적인 오일러 각의 공식을 사용하면 적분을 하지 않아도 각도를 계산할 수 있다. 따라서 적분에의한 누적오차는 발생하지는 않았다. 다만 측정한 값이 실제 값과 다른 특성을 확인 할 수 있었다.

정리하면, 가속도계의 오차는 적분을 하지 않기 때문에 오차가 발산하지는 않지만 정확한 각도를 측정할 수 없었고, 반대로 자이로계는 누적오차로 인해 값이 발산하지만, 순간순간 얻은 각도 값은 정확한 값이었다. 
이 특성을 이용하여 가속도계로 측정한 자세를 칼만필터의 측정값으로 선정한다. 칼만필터는 이값을 각속도를 적분하여 계산한 자세와 비교하여 오차를 보정한다. 여기부터~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


### - 오일러각과 쿼터니언

오일러각을 상태 변수로 잡고 시스템 모델을 선정하게 되면 시스템 모델 안에 있는 오일러 각을 빼낼 수 없다. 이것을 해결하기 위해 쿼터니언 회전 변환을 이용하여 각속도와의 관계를 다음과 같이 정의한다. 


<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\bg_white&space;\frac{1}{2}&space;\times&space;\begin{bmatrix}0&&space;&space;-p&&space;&space;-q&&space;&space;-r\\&space;p&&space;&space;0&&space;&space;r&&space;&space;-q\\&space;q&&space;&space;-r&&space;&space;0&&space;&space;p\\&space;r&&space;&space;q&&space;&space;-p&&space;&space;0\\\end{bmatrix}" title="\bg_white \frac{1}{2} \times \begin{bmatrix}0& -p& -q& -r\\ p& 0& r& -q\\ q& -r& 0& p\\ r& q& -p& 0\\\end{bmatrix}" /> (쿼터니언과 각속도의 관계)

<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\bg_white&space;I&space;&plus;&space;\triangle&space;dt\cdot&space;\frac{1}{2}&space;\times&space;\begin{bmatrix}0&&space;&space;-p&&space;&space;-q&&space;&space;-r\\&space;p&&space;&space;0&&space;&space;r&&space;&space;-q\\&space;q&&space;&space;-r&&space;&space;0&&space;&space;p\\&space;r&&space;&space;q&&space;&space;-p&&space;&space;0\\\end{bmatrix}" title="\bg_white I + \triangle dt\cdot \frac{1}{2} \times \begin{bmatrix}0& -p& -q& -r\\ p& 0& r& -q\\ q& -r& 0& p\\ r& q& -p& 0\\\end{bmatrix}" /> (이산 시스템으로 변환)

```c++
void initSystemMatrix(){
     Matrix4d temp; //GyX == p, GyY == q, GyZ ==r;
     temp(0,0) = 0.0; temp(0,1) = -GyX; temp(0,2) = -GyY; temp(0,3) = -GyZ;
     temp(1,0) = GyX; temp(1,1) = 0.0; temp(1,2) = GyZ; temp(1,3) = -GyY;
     temp(2,0) = GyY; temp(2,1) = -GyZ; temp(2,2) = 0.0; temp(2,3) = GyX;
     temp(3,0) = GyZ; temp(3,1) = GyY; temp(3,2) = -GyX; temp(3,3) = 0.0;
     A = temp*dt*1/2 + I;
}
```

시스템 모델의 상태변수는 쿼터니언이기 때문에 가속도로 계산한 오일러각도를 쿼터니언 공식으로 바꿔줘야 한다.

<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\bg_white&space;\begin{Bmatrix}&space;q_{1}\\&space;q_{2}\\&space;q_{3}\\&space;q_{4}\end{Bmatrix}&space;=&space;\begin{Bmatrix}cos(\frac{\phi}{2})cos(\frac{\theta}{2})cos(\frac{\psi}{2})&space;&plus;&space;sin(\frac{\phi}{2})sin(\frac{\theta}{2})sin(\frac{\psi}{2})&space;\\&space;sin(\frac{\phi}{2})cos(\frac{\theta}{2})cos(\frac{\psi}{2})&space;-&space;cos(\frac{\phi}{2})sin(\frac{\theta}{2})sin(\frac{\psi}{2})&space;&space;\\&space;cos(\frac{\phi}{2})sin(\frac{\theta}{2})cos(\frac{\psi}{2})&space;&plus;&space;sin(\frac{\phi}{2})cos(\frac{\theta}{2})sin(\frac{\psi}{2})&space;\\&space;cos(\frac{\phi}{2})cos(\frac{\theta}{2})sin(\frac{\psi}{2})&space;-&space;sin(\frac{\phi}{2})sin(\frac{\theta}{2})cos(\frac{\psi}{2})\end{Bmatrix}" title="\bg_white \begin{Bmatrix} q_{1}\\ q_{2}\\ q_{3}\\ q_{4}\end{Bmatrix} = \begin{Bmatrix}cos(\frac{\phi}{2})cos(\frac{\theta}{2})cos(\frac{\psi}{2}) + sin(\frac{\phi}{2})sin(\frac{\theta}{2})sin(\frac{\psi}{2}) \\ sin(\frac{\phi}{2})cos(\frac{\theta}{2})cos(\frac{\psi}{2}) - cos(\frac{\phi}{2})sin(\frac{\theta}{2})sin(\frac{\psi}{2}) \\ cos(\frac{\phi}{2})sin(\frac{\theta}{2})cos(\frac{\psi}{2}) + sin(\frac{\phi}{2})cos(\frac{\theta}{2})sin(\frac{\psi}{2}) \\ cos(\frac{\phi}{2})cos(\frac{\theta}{2})sin(\frac{\psi}{2}) - sin(\frac{\phi}{2})sin(\frac{\theta}{2})cos(\frac{\psi}{2})\end{Bmatrix}" />
