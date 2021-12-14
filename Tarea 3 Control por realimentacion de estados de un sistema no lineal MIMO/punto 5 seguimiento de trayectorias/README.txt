Cntrol II Tarea 3

5) Seguimiento de trayectoria.

Mediante un script de MatLab se realiza el seguimiento de una curva parametrizada en el tiempo.
Se considera como salida las coordenadas de un punto desplazado una distancia 'l' (se cambió de nombre la variable para concordar con la imagen usada en el reporte de la tarea) del centro de rotación del robot de ruedas de control diferencial.
El script integra funciones para:
	- Aplicación de la ley de control (tracking.m)
	- Visualización de resultados (plot_results.m)
	- Visualización de animación de resultados (animation.m)
	
Ejecución:
Ejecutando el script main.m se lleva a cabo el seguimiento de trayectoria a partir de tres condiciones iniciales propuestas, el tiempo tau y el destino xf (común para todos las condiciones iniciales), asi como la visualización de los resultados.
Para visualizar la animación es necesario ejecutar el script animation.m el cual requiere como entradas (X, Y, T, dt). Por ejemplo, si se desea visualizar la animación para los resultados de la primer condición inicial se debe ejecutar '>> animation(X1, Y1, T, dt)'
