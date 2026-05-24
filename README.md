# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Asignatura:** Robótica y Sistemas Autónomos
**Integrantes:**
* Sebastian Cruz
* Joaquin Fuenzalida 
* Ignacio ávila

## 1. Objetivo
Implementar un sistema básico de navegación reactiva en Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda, aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

## 2. Descripción del Robot y Sensores
Para este laboratorio se utilizó el robot diferencial **e-puck**. La percepción del entorno y el cálculo de la odometría se basan en los siguientes sensores integrados:
* **Sensores de distancia infrarrojos (IR):** Se habilitaron los 8 sensores del e-puck (`ps0` a `ps7`). 
    * **Frontales:** Se combinaron las lecturas de `ps0`, `ps7`, `ps1` y `ps6` (tomando el valor máximo) para representar el obstáculo frontal más crítico de forma unificada.
    * **Laterales:** Se utilizaron `ps5` y `ps6` para el lado izquierdo, y `ps1` y `ps2` para el lado derecho, con el fin de calcular el diferencial de distancia lateral.
* **Encoders de rueda:** Se habilitaron los sensores angulares `left wheel sensor` y `right wheel sensor` para medir el desplazamiento en radianes de cada rueda.

## 3. Frecuencia de Muestreo
El ciclo de control y la toma de datos están sincronizados con el paso de simulación de Webots (`TIME_STEP`):
* **Tiempo de muestreo ($T_{s}$):** $0.064$ s (64 ms)
* **Frecuencia de muestreo ($f_{s}$):** $15.625$ Hz

## 4. Análisis de Señales Registradas
*(Compañeros: Aquí analizar brevemente el gráfico de las señales crudas de los sensores IR. Mencionen el ruido o las fluctuaciones observadas durante las pruebas).*

## 5. Estimación y Filtrado

### 5.1 Estimación del Avance mediante Encoders
Los encoders entregan la posición angular acumulada en radianes. Sabiendo que el radio de las ruedas del e-puck es $r = 0.0205$ m, el desplazamiento lineal del robot en cada ciclo ($\Delta s$) se estimó calculando el promedio del avance del arco de ambas ruedas:

$$\Delta s = r \frac{\Delta \theta_{L} + \Delta \theta_{R}}{2}$$

### 5.2 Filtro Simple
Para suavizar las lecturas crudas y ruidosas de los sensores frontales, se implementó un filtro de **Media Móvil Exponencial (EMA)**. Se definió un coeficiente $\alpha = 0.3$, dándole un peso moderado a la medición actual y conservando el histórico reciente, mediante la siguiente ecuación:

$$EMA_{k} = \alpha \cdot \text{raw}_{k} + (1 - \alpha) \cdot EMA_{k-1}$$

### 5.3 Implementación del Filtro de Kalman
Para obtener una estimación robusta y estable de la distancia frontal, se fusionó la odometría (predicción) con la lectura de los sensores IR (corrección).

**1. Etapa de Predicción:** Se estimó el aumento en la señal del sensor IR asumiendo que el robot avanza frontalmente hacia un obstáculo. Se aplicó un factor de escala (`IR_SCALE = 500.0`) para convertir los metros avanzados a unidades brutas del sensor IR. Esta etapa solo se ejecuta cuando el robot avanza de frente (`FORWARD` o `ESCAPE`).

$$\hat{d}_{k}^{-} = \hat{d}_{k-1} + \text{IR\_SCALE} \cdot \max(\Delta s, 0)$$
$$P^{-} = P_{k-1} + Q$$
*(Donde la varianza del proceso se configuró en $Q = 5.0$)*

**2. Etapa de Corrección:** Se actualizó la estimación utilizando la medición real (`raw`) de los sensores frontales.

$$K = \frac{P^{-}}{P^{-} + R}$$
$$\hat{d}_{k} = \hat{d}_{k}^{-} + K(z_{k} - \hat{d}_{k}^{-})$$
$$P = (1 - K)P^{-}$$
*(Donde la varianza de la medición se configuró en $R = 200.0$, asumiendo alto ruido en los sensores infrarrojos).*

## 6. Lógica de Navegación Reactiva
La toma de decisiones recae en una máquina de estados finitos alimentada por la variable estimada por el filtro de Kalman ($\hat{d}_{k}$). 

1.  **Avance (`FORWARD`):** El robot avanza en línea recta a $5.0$ rad/s. Si la estimación frontal supera el umbral de proximidad (`THRESHOLD_HIGH = 95.0`), se detiene el avance y evalúa el giro.
2.  **Decisión de Giro (`get_turn_direction`):** Se calcula la diferencia entre las mediciones laterales (`izquierdas - derechas`). 
    * Si el obstáculo está más cerca por la izquierda, el estado cambia a `TURN_RIGHT`.
    * Si está más cerca por la derecha, el estado cambia a `TURN_LEFT`.
    * Se implementó un desempate de seguridad usando los sensores frontales extremos si la diferencia lateral es imperceptible.
3.  **Ejecución de Giro:** Se detiene la rueda interior ($0.0$ rad/s) y se hace girar la exterior a $3.5$ rad/s durante 10 ciclos de reloj (`TURN_DURATION`).
4.  **Evasión (`ESCAPE`):** Tras girar, el robot fuerza un avance recto durante 8 ciclos (`ESCAPE_DURATION`) para separarse del obstáculo antes de reiniciar su evaluación frontal, evitando bucles infinitos de giro en esquinas.

## 7. Gráficos de Señales
*(Compañeros: Insertar aquí los gráficos exportados de la simulación. Reemplazar las líneas de abajo con las rutas de sus imágenes).*

![Gráfico de Comparación de Señales](images/grafico_senales.png)

## 8. Resultados en Escenarios de Prueba
*(Compañeros: Describir brevemente cómo se comportó el robot en los mundos creados).*

* **Escenario Simple (`escenario_simple.wbt`):** [Analizar estabilidad y si evitó colisiones con pocos obstáculos].
* **Escenario Complejo (`escenario_complejo.wbt`):** [Analizar la capacidad para evadir laberintos de cajas y si la lógica de escape fue efectiva].

## 9. Conclusiones
*(Compañeros: Redactar un párrafo de cierre analizando la efectividad de Kalman frente a las mediciones crudas).*

## 10. Instrucciones de Ejecución
1. Clonar este repositorio.
2. Abrir la aplicación Webots (Versión R2025a recomendada).
3. Cargar el mundo deseado (`escenario_simple.wbt` o `escenario_complejo.wbt`) ubicado en la carpeta `worlds`.
4. Verificar que el controlador del robot `e-puck` esté asignado como `lab2_controller`.
5. Ejecutar la simulación presionando el botón "Play".
