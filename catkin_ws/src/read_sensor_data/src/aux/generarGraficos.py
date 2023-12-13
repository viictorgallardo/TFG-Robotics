import matplotlib.pyplot as plt
import math
import os
#IMPRIME EN UN GRAFICO LAS COORDENADAS DE LOS 3 ROBOTS DURANTE EL TIEMPO

#Listas para almacenar las coordenadas x e y
coordenadas_x = []
coordenadas_y = []

coordenadas_x_1 = []
coordenadas_y_1 = []

coordenadas_x_2 = []
coordenadas_y_2 = []

coordenadas_x_3 = []
coordenadas_y_3 = []


distancias1 = []
distancias2 = []
distancias3 = []


mu_0 = []
mu_1 = []
mu_2 = []

#ERRORES DE POSICION

erroresX0 = []
erroresY0 = []

erroresX1 = []
erroresY1 = []

erroresX2 = []
erroresY2 = []




directorio_actual = os.getcwd()

# Ruta del archivo de coordenadasRO.txt en el directorio actual
ruta_coordenadas_ro = os.path.join(directorio_actual, 'coordenadasR0.txt')

ruta_coordenadas_r1 = os.path.join(directorio_actual, 'coordenadasR1.txt')

ruta_coordenadas_r2 = os.path.join(directorio_actual, 'coordenadasR2.txt')

ruta_coordenadas_r3 = os.path.join(directorio_actual, 'coordenadasR3.txt')

medidas_path = os.path.join(directorio_actual, 'medidas.txt')

salida_logs = os.path.join(directorio_actual, 'salida_logs.txt')

distancias = os.path.join(directorio_actual, 'distancias.txt')




medidas = []

#Lee las coordenadas del archivo
with open(medidas_path, 'r') as archivo:
    for linea in archivo:
        valor = float(linea.strip())  # Convierte el valor de la línea a un número flotante
        medidas.append(valor)

# Genera el gráfico de dispersión
#plt.scatter(range(250), medidas[0:250], label='Medidas', color='red')
plt.scatter(range(len(medidas)), medidas, label='Medidas', color='red')

# Personalización del gráfico
plt.title('Medidas')
plt.xlabel('Índice')
plt.ylabel('Valor')
plt.legend()

plt.show()



#Lee las coordenadas del archivo
with open(ruta_coordenadas_ro, 'r') as archivo:
    for linea in archivo:
        x, y, _, _ , errorX, errorY = map(float, linea.strip().split(','))
        coordenadas_x.append(x)
        coordenadas_y.append(y)
        erroresX0.append(errorX)
        erroresY0.append(errorY)
        


#Lee las coordenadas del archivo
with open(ruta_coordenadas_r1, 'r') as archivo:
    for linea in archivo:
        x, y, _, _ , errorX, errorY = map(float, linea.strip().split(','))
        coordenadas_x_1.append(x)
        coordenadas_y_1.append(y)
        erroresX1.append(errorX)
        erroresY1.append(errorY)



#Lee las coordenadas del archivo
with open(ruta_coordenadas_r2, 'r') as archivo:
    for linea in archivo:
        x, y, _, _ ,errorX, errorY = map(float, linea.strip().split(','))
        coordenadas_x_2.append(x)
        coordenadas_y_2.append(y)
        erroresX2.append(errorX)
        erroresY2.append(errorY)

# #Lee las coordenadas del archivo
#with open(ruta_coordenadas_r3, 'r') as archivo:
 #    for linea in archivo:
  #       x, y, _, _ = map(float, linea.strip().split(','))
   #      coordenadas_x_3.append(x)
    #     coordenadas_y_3.append(y)


#Crea un gráfico de dispersión con las coordenadas
plt.scatter(coordenadas_x, coordenadas_y, label='Coordenadas 0', color='blue')
plt.scatter(coordenadas_x_1, coordenadas_y_1, label='Coordenadas 1', color='green')
plt.scatter(coordenadas_x_2, coordenadas_y_2, label='Coordenadas 2', color='red')
#plt.scatter(coordenadas_x_3, coordenadas_y_3, label='Coordenadas 3', color='yellow')

#Personaliza el gráfico si lo deseas
plt.title('Gráfico de Coordenadas')
plt.xlabel('Eje X')
plt.ylabel('Eje Y')
plt.legend()

#Muestra el gráfico
plt.show()


cuenta = 0

with open(salida_logs, 'r') as archivo1:
    for linea in archivo1:

        if linea.startswith("ITERACION"):
            continue

        palabras = linea.split()

        # Obtiene el último número de la línea y lo convierte a float
        ultimo_numero = float(palabras[-1])

        # Decide en qué vector guardar el último número basado en el índice del robot
        indice_robot = int(palabras[2])  # Obtener el índice del robot de la línea
        if indice_robot == 0:
            mu_0.append(ultimo_numero)
            cuenta = cuenta + 1
        elif indice_robot == 1:
            mu_1.append(ultimo_numero)
        elif indice_robot == 2:
            mu_2.append(ultimo_numero)


iteraciones_totales = range(0,cuenta)
        
#Crea un gráfico de dispersión con las coordenadas
plt.scatter( iteraciones_totales, mu_0, label='Mu 0', color='blue')
plt.scatter( iteraciones_totales, mu_1, label='Mu 1', color='green')
plt.scatter( iteraciones_totales, mu_2, label='Mu 2', color='red')

#Personaliza el gráfico si lo deseas
plt.title('Gráfico de Repulsión')
plt.xlabel('Eje X')
plt.ylabel('Eje Y')
plt.legend()

#Muestra el gráfico
plt.show()





cuenta = 0
#Lee las coordenadas del archivo
with open(distancias, 'r') as archivo:
    for linea in archivo:
        x, y = map(float, linea.strip().split(','))
        distancias1.append(x)
        distancias2.append(y)
        #distancias3.append(z)
        cuenta = cuenta + 1



iteraciones_totales = range(0,cuenta)
        
#Crea un gráfico de dispersión con las coordenadas
plt.scatter( iteraciones_totales, distancias1, label='W2 - W3', color='red')
plt.scatter( iteraciones_totales, distancias2, label='W3 - W4', color='green')
#plt.scatter( iteraciones_totales, distancias3, label='W4 - W1', color='red')

#Personaliza el gráfico si lo deseas
plt.title('Gráfico de distancias en el Platooning')
plt.xlabel('Eje X')
plt.ylabel('Eje Y')
plt.legend()

#Muestra el gráfico
plt.show()


#ERRORES DE POSICION
print(len(erroresX0))
print(len(erroresX1))
print(len(erroresX2))
print(len(erroresY0))
print(len(erroresY1))
print(len(erroresY2))


# Gráfico de dispersión para errores en el eje X
plt.scatter(range(len(erroresX0)), erroresX0, label='ErrorX0', color='red')
plt.scatter(range(len(erroresX1)), erroresX1, label='ErrorX1', color='green')
plt.scatter(range(len(erroresX2)), erroresX2, label='ErrorX2', color='blue')

# Personalización del gráfico
plt.title('Errores en el eje X')
plt.xlabel('Índice')
plt.ylabel('Valor')
plt.legend()

# Mostrar el gráfico
plt.show()

# Suponiendo que erroresY0, erroresY1 y erroresY2 son listas de datos

# Gráfico de dispersión para errores en el eje Y
plt.scatter(range(len(erroresY0)), erroresY0, label='ErrorY0', color='red')
plt.scatter(range(len(erroresY1)), erroresY1, label='ErrorY1', color='green')
plt.scatter(range(len(erroresY2)), erroresY2, label='ErrorY2', color='blue')

# Personalización del gráfico
plt.title('Errores en el eje Y')
plt.xlabel('Índice')
plt.ylabel('Valor')
plt.legend()

# Mostrar el gráfico
plt.show()


