Cntrol II Tarea 3

2)Estabilización del sistema al origen.

Mediante un script de MatLab se realiza la estabilización de las salidas al origen de un DDR.  
Se considera como salida las coordenadas de un punto desplazado una distancia 'l' (se cambió de nombre la variable para concordar con la imagen usada en el reporte de la tarea) del centro de rotación del robot de ruedas de control diferencial.
El script integra funciones para:
	- Aplicación de la ley de control (estab_salidas.m)
	- Visualización de resultados (plot_results.m)
	- Visualización de animación de resultados (animation.m)
	
Ejecución:
Ejecutando el script main.m se lleva a cabo la estabilización del sistema a partir de tres condiciones iniciales propuestas, asi como la visualización de los resultados.
Para visualizar la animación es necesario ejecutar el script animation.m el cual requiere como entradas (X, Y, T, dt). Por ejemplo, si se desea visualizar la animación para los resultados de la primer condición inicial se debe ejecutar '>> animation(X1, Y1, T, dt)'
