## Ejercicio 2

Crear un nodo que recibe un texto como action server y envía cada palabra del texto como feedback a 1Hz.
Crear otro nodo como action client que reciba un texto como argumento y lo envíe al primer nodo como action. Se subscribe al feedback del primero y lo muestra en la terminal. Cuando el primer nodo indica que terminó, el segundo nodo publica el mensaje “Texto republicado!” Cree el mensaje custom para hacerlo.
Crear un roslaunch que permita pasar el texto como argumento y ejecute ambos
nodos.

## Autores
- José Luis Krüger
- Juan Manuel Guariste

## Ejemplo de ejecución

```bash
ros2 launch kruger_guariste_ej_2 ej.launch.py text:="texto de prueba para validar funcionamiento"
```

