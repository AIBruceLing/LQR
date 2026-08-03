

# Cálculo de curvatura
![image](https://github.com/user-attachments/assets/0ed3c884-ef34-4948-8e55-f71b306de4a9)

# Deducción de LQR
![image](https://github.com/user-attachments/assets/951b7a1c-5401-4211-a0dc-34776080d6b2)
![image](https://github.com/user-attachments/assets/5a936b0a-248c-450a-99f8-ac9ca944cd13)


# Método iterativo para resolver la ecuación de Riccati
![image](https://github.com/user-attachments/assets/a671411d-ee5e-40ae-b224-d94c746d2a47)


# Comandos
```shell
mkdir build
cd build/
cmake ..
make
``` 

## Ejecución
``` shell
cd build/
./lqr_controller    //lqr
./lqr_pid_controller  //lqr + pid
```
## Resultados
### LQR   (Velocidad constante 2 m/s)
![image](https://github.com/user-attachments/assets/9365fa22-cf6b-432c-b75e-4718139d3e1c)
### LQR + PID  (Velocidad inicial 0, velocidad objetivo 4 m/s)
![image](https://github.com/user-attachments/assets/f3d510fe-dae4-45bb-865e-7d8781b47088)
