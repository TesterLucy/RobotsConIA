# 🤖 Sistema de Percepción Inteligente para Robots Futbolistas con ESP32

## 📖 Descripción

Este proyecto desarrolla un **sistema de percepción inteligente para robots futbolistas autónomos**, implementado sobre una plataforma **ESP32** y diseñado para entornos de fútbol robótico similares a la RoboCup Small Size League (SSL).

El sistema permite que el robot identifique objetos presentes en el campo de juego, como balones, oponentes y obstáculos, utilizando sensores ultrasónicos y sensores de color. La información capturada es procesada mediante una **Red Neuronal Perceptrón Multicapa (MLP)**, complementada con técnicas de inteligencia artificial basadas en **Algoritmos Genéticos (GA)** y **Optimización por Enjambre de Partículas (PSO)** para la toma de decisiones en tiempo real.

---

## 🎯 Objetivos

* Diseñar un sistema de percepción inteligente para robots móviles.
* Detectar y clasificar objetos presentes en el campo de juego.
* Implementar aprendizaje supervisado mediante redes neuronales.
* Optimizar la toma de decisiones utilizando algoritmos bioinspirados.
* Ejecutar inferencia en tiempo real sobre hardware embebido ESP32.
* Mejorar la navegación autónoma y la interacción con el balón.

---

## ⚙️ Características Principales

* Plataforma embebida basada en ESP32.
* Procesamiento en tiempo real con ciclos de aproximadamente 30 ms.
* Clasificación inteligente de escenarios mediante IA.
* Arquitectura híbrida combinando Machine Learning y algoritmos evolutivos.
* Navegación autónoma y evasión de obstáculos.
* Sistema adaptable a condiciones dinámicas del entorno.

---

## 🔧 Hardware Utilizado

### Microcontrolador

* ESP32

### Sensores

* 3 Sensores ultrasónicos HC-SR04
* 1 Sensor de color TCS34725

### Variables Capturadas

#### Distancias

* Distancia frontal superior
* Distancia frontal inferior
* Distancia izquierda
* Distancia derecha

#### Variaciones Temporales

* Cambio de distancia entre lecturas consecutivas

#### Variables de Color

* Componente Roja (R)
* Componente Verde (G)
* Componente Azul (B)

---

## 🧠 Inteligencia Artificial Implementada

### Red Neuronal Perceptrón Multicapa (MLP)

El sistema utiliza una red neuronal con arquitectura:

```text
9 → 12 → 4
```

Donde:

* 9 neuronas de entrada
* 12 neuronas ocultas
* 4 neuronas de salida

### Clases Identificadas

| Clase | Objeto Detectado  |
| ----- | ----------------- |
| 0     | Pared u obstáculo |
| 1     | Oponente          |
| 2     | Balón libre       |
| 3     | Balón disputado   |

### Funciones Utilizadas

* Normalización Min-Max
* Activación Sigmoide
* Softmax
* Entropía Cruzada

---

## 🧬 Algoritmos Bioinspirados

### Optimización por Enjambre de Partículas (PSO)

El algoritmo PSO se utiliza para:

* Exploración del entorno.
* Optimización de trayectorias.
* Búsqueda eficiente de objetivos.
* Navegación dinámica.

### Algoritmos Genéticos (GA)

Los algoritmos genéticos permiten:

* Ajuste automático de parámetros.
* Adaptación a condiciones cambiantes.
* Optimización del comportamiento del robot.
* Evolución de estrategias de juego.

### Arquitectura Híbrida

El sistema combina:

```text
GA → Estrategia Global
PSO → Movimiento Local
MLP → Percepción Inteligente
```

Esta integración permite una toma de decisiones más robusta y adaptable.

---

## 🔄 Máquina de Estados Finitos

El comportamiento del robot está gobernado por una máquina de estados compuesta por:

### BUSCAR

Explora el entorno en busca de objetivos.

### ACERCARSE

Se dirige hacia el objeto detectado.

### VERIFICAR_COLOR

Confirma si el objeto corresponde al balón.

### EMPUJAR

Interactúa físicamente con el balón.

### EVITAR

Realiza maniobras de evasión ante obstáculos.

---

## 📊 Flujo de Procesamiento

```text
Lectura de Sensores
        ↓
Cálculo de Distancias
        ↓
Preprocesamiento
        ↓
Normalización
        ↓
Red Neuronal MLP
        ↓
Clasificación
        ↓
Máquina de Estados
        ↓
Acción del Robot
```

---

## 🚀 Resultados Obtenidos

Durante las pruebas realizadas se observaron los siguientes beneficios:

* Detección confiable del balón naranja.
* Mayor precisión en la clasificación de objetos.
* Reducción significativa de colisiones.
* Mejora en la búsqueda de objetivos mediante PSO.
* Adaptación progresiva gracias a los Algoritmos Genéticos.
* Inferencia eficiente ejecutada directamente en el ESP32.

---

## 📚 Fundamentos Matemáticos

El proyecto incorpora conceptos de:

* Redes Neuronales Artificiales.
* Aprendizaje Supervisado.
* Normalización Estadística.
* Optimización Bioinspirada.
* Sistemas Embebidos.
* Robótica Móvil.
* Máquinas de Estados Finitos.
* Inteligencia Artificial Aplicada.

---

## 🎓 Contexto Académico

Proyecto desarrollado para la asignatura **Toma de Decisiones** del programa de **Ciencias de la Computación e Inteligencia Artificial** de la Universidad Sergio Arboleda.

---

## 👨‍💻 Autores

* Cristian León
* Santiago Rodríguez
* Juan Pablo Chavarro

Universidad Sergio Arboleda
Escuela de Ciencias Exactas e Ingeniería
Ciencias de la Computación e Inteligencia Artificial
