Cntrol II Tarea 3

4) Seguimiento de una línea recta.

Mediante un script de MatLab se realiza el seguimiento de una línea recta que conecta las coordenadas de un punto desplazado del orign del DDR a partir de la configuración inicial de éste último hacia un punto xf. El tiempo de recorrido de esta línea debe realizarse en un tiempo tau.
Se considera como salida las coordenadas de un punto desplazado una distancia 'l' (se cambió de nombre la variable para concordar con la imagen usada en el reporte de la tarea) del centro de rotación del robot de ruedas de control diferencial.
El script integra funciones para:
	- Aplicación de la ley de control (line_track.m)
	- Visualización de resultados (plot_results.m)
	- Visualización de animación de resultados (animation.m)
	
Ejecución:
Ejecutando el script main.m se lleva a cabo el seguimeinto de una línea recta a partir de tres condiciones iniciales propuestas, el tiempo tau y el destino xf (común para todos las condiciones iniciales), asi como la visualización de los resultados.
Para visualizar la animación es necesario ejecutar el script animation.m el cual requiere como entradas (X, Y, T, dt). Por ejemplo, si se desea visualizar la animación para los resultados de la primer condición inicial se debe ejecutar '>> animation(X1, Y1, T, dt)'
