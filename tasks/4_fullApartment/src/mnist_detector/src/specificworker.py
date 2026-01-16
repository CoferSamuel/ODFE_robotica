#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import traceback
import math
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms

from PySide6 import QtCore
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console

# ---------------------------------------------------------
# Configuración e Importaciones
# ---------------------------------------------------------
# Añadimos la ruta de los ficheros generados por robocompdsl a sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), '../generated'))
try:
    from genericworker import *
    import interfaces as ifaces
except ImportError:
    print("Error: No se han encontrado los ficheros generados de RoboComp.")
    sys.exit(-1)

# Inicializamos la consola para logs bonitos
console = Console(highlight=False)

# ---------------------------------------------------------
# Definición de la Red Neuronal (CNN)
# ---------------------------------------------------------
# Clase que define la arquitectura de la red neuronal convolucional para MNIST
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        # Capa convolucional 1: Entrada 1 canal (Grayscale), Salida 32 canales, Kernel 3x3
        self.conv1 = nn.Conv2d(1, 32, 3, 1)
        # Capa convolucional 2: Entrada 32 canales, Salida 64 canales, Kernel 3x3
        self.conv2 = nn.Conv2d(32, 64, 3, 1)
        # Capas totalmente conectadas (Fully Connected)
        # 64 canales * 12 * 12 (tamaño imagen tras pooling) -> 128 neuronas
        self.fc1 = nn.Linear(64 * 12 * 12, 128)
        # 128 neuronas -> 10 salidas (dígitos 0-9)
        self.fc2 = nn.Linear(128, 10)

    # Función que define el paso hacia adelante (Forward Pass)
    def forward(self, x):
        x = self.conv1(x)
        x = F.relu(x)        # Activación ReLU
        x = self.conv2(x)
        x = F.relu(x)        # Activación ReLU
        x = F.max_pool2d(x, 2) # Max Pooling 2x2 para reducir dimensionalidad
        x = torch.flatten(x, 1) # Aplanar tensor para capa lineal
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        output = F.log_softmax(x, dim=1) # Log Softmax para obtener probabilidades logarítmicas
        return output

# ---------------------------------------------------------
# Clase SpecificWorker
# ---------------------------------------------------------
# Clase principal del componente que hereda de GenericWorker
class SpecificWorker(GenericWorker, ifaces.RoboCompMNIST.MNIST):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        # Variables compartidas para almacenar el último resultado
        self.latest_digit = -1
        self.latest_confidence = 0.0
        self.last_request_time = 0.0
        self.current_period = 500

        # --- Cargar Modelo DNN ---
        # Seleccionar dispositivo: CUDA (GPU) si está disponible, si no CPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Net().to(self.device)
        
        # Ruta absoluta al archivo de pesos entrenados (.pt)
        # specificworker.py está en src/mnist_detector/src/
        # my_network.pt está en src/ (del proyecto C++)
        # Así que subimos: src/ -> mnist_detector/ -> src/ (del proyecto) -> raiz component -> my_network.pt ??
        # No, la estructura es: tasks/4_fullApartment/src/my_network.pt
        # __file__ es .../tasks/4_fullApartment/src/mnist_detector/src/specificworker.py
        # .. -> .../tasks/4_fullApartment/src/mnist_detector/
        # ../.. -> .../tasks/4_fullApartment/src/
        # ../../.. -> .../tasks/4_fullApartment/ (Si my_network.pt estuviera aqui)
        
        # EL USUARIO TIENE my_network.pt en src/DNN/my_network.pt
        # specificworker.py está en src/mnist_detector/src/
        # ../.. nos lleva a src/ (del proyecto tareas)
        # ../../DNN/my_network.pt es la ruta correcta
        model_path = os.path.join(os.path.dirname(__file__), '../../DNN/my_network.pt')
        
        try:
            if os.path.exists(model_path):
                # Cargar los pesos en el modelo
                self.model.load_state_dict(torch.load(model_path, map_location=self.device))
                self.model.eval() # Poner modelo en modo evaluación (no entrenamiento)
                print(f"Modelo cargado correctamente en {self.device} desde {model_path}")
            else:
                print(f"ERROR CRITICO: No se encuentra el modelo en {model_path}")
        except Exception as e:
            print(f"Excepción cargando modelo: {e}")
            traceback.print_exc()

        # --- Transformaciones de Imagen ---
        # Preprocesamiento necesario para que la imagen coincida con el formato de entrenamiento MNIST
        self.transform = transforms.Compose([
            transforms.ToTensor(), # Convertir a Tensor de PyTorch
            transforms.Normalize((0.1307,), (0.3081,)) # Normalizar con media/desviación de MNIST
        ])

        if startup_check:
            self.startup_check()
        else:
            self.init_camera()
            self.timer.timeout.connect(self.compute) # Conectar timer al método compute
            self.timer.start(self.current_period) # Start at 500ms (IDLE)

    # Inicialización y conexión robusta con la cámara
    def init_camera(self):
        started_camera = False
        while not started_camera:
            try:
                print("Conectando a Camera360RGB...")
                # Intentar obtener una imagen dummy para verificar conexión
                self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                print(f"Cámara conectada. Res: {self.rgb_original.width}x{self.rgb_original.height}")
                started_camera = True
            except Exception as e:
                print(f"Esperando a la cámara... Error: {e}")
                time.sleep(1)

    # ---------------------------------------------------------
    # Implementación de la Interfaz Ice (Servidor)
    # ---------------------------------------------------------
    
    # Este método es llamado remotamente por el cliente C++ (multiroom)
    def getNumber(self, current=None):
        self.last_request_time = time.time()
        # If we were idling, we don't restart the timer here to avoid thread-safety issues with Qt,
        # but the next 'compute' will switch to FAST mode.
        try:
            res = ifaces.RoboCompMNIST.Digit()
            res.val = int(self.latest_digit)      # Devolver último dígito detectado
            res.confidence = float(self.latest_confidence) # Devolver confianza
            return res
        except Exception as e:
            # Si hay error, devolvemos -1 y 0.0
            traceback.print_exc()
            res = ifaces.RoboCompMNIST.Digit()
            res.val = -1
            res.confidence = 0.0
            return res

    # ---------------------------------------------------------
    # Bucle Principal de Cómputo (compute)
    # ---------------------------------------------------------
    # Se ejecuta periódicamente según el Period definido
    @QtCore.Slot()
    def compute(self):
        # Dynamically adjust period based on requests
        now = time.time()
        # If there was a request in the last 2 seconds, run at 50ms
        if now - self.last_request_time < 2.0:
            new_period = 50
        else:
            new_period = 500
            
        if new_period != self.current_period:
            self.current_period = new_period
            self.timer.setInterval(self.current_period)
            if self.current_period == 50:
                print("DEBUG: MNIST Mode -> FAST (50ms)")
            else:
                print("DEBUG: MNIST Mode -> IDLE (500ms)")

        try:
            # 1. Obtener imagen de la cámara (WebotsBridge)
            image = self.camera360rgb_proxy.getROI(0, 0, 0, 0, 0, 0)
            
            # Debug struct content
            # print(f"DEBUG: TImage -> Compressed: {image.compressed}, CamID: {image.cameraID}, W: {image.width}, H: {image.height}, Depth: {image.depth}")
            # print(f"DEBUG: sizes: {len(image.image)} bytes")
            
            # Debug struct content
            print(f"DEBUG: TImage -> Compressed: {image.compressed}, CamID: {image.cameraID}, W: {image.width}, H: {image.height}, Depth: {image.depth}")
            print(f"DEBUG: ImgType len: {len(image.image)}")
            
            # Verificar validez de la imagen antes de procesar
            # Verificar validez de la imagen antes de procesar
            if image.width <= 0 or image.height <= 0 or len(image.image) == 0:
                print(f"Advertencia: Imagen inválida recibida (W:{image.width}, H:{image.height}, Len:{len(image.image)})")
                return

            # Convertir buffer de bytes a array numpy (Imagen OpenCV)
            if image.compressed:
                np_arr = np.frombuffer(image.image, np.uint8)
                color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if color is None:
                    print("Error: Fallo al decodificar imagen comprimida")
                    return
            else:
                color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
            
            # 2. Hacer copia para pintar resultados visuales
            color_copy = color.copy()

            # 3. Detectar si hay un panel (recuadro negro) en la imagen
            # Usa visión artificial clásica antes de llamar a la IA
            rect = self.detect_frame(color)

            if rect is not None:
                x1, y1, x2, y2 = rect
                
                # Extraer la Región de Interés (ROI) del panel
                roi = color[y1:y2, x1:x2]
                if roi.size > 0:
                    # --- Pre-procesado para MNIST ---
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) # A escala de grises
                    # Redimensionar a 28x28 píxeles (formato de entrada de la red)
                    resized_roi = cv2.resize(gray_roi, (28, 28), interpolation=cv2.INTER_AREA)
                    # MNIST espera fondo negro y letra blanca.
                    # Nuestra imagen (panel) tiene fondo BLANCO y letra NEGRA.
                    # Al invertir (bitwise_not), obtenemos Fondo NEGRO y Letra BLANCA (Correcto para MNIST)
                    inverted_roi = cv2.bitwise_not(resized_roi)

                    # --- Inferencia (Predicción) ---
                    # Convertir a tensor, añadir dimensión de batch (unsqueeze) y enviar a device
                    input_tensor = self.transform(inverted_roi).unsqueeze(0).to(self.device)
                    
                    with torch.no_grad(): # Desactivar gradientes para inferencia (optimización)
                        output = self.model(input_tensor)
                        probs = torch.exp(output) # Convertir log_probs a probabilidades reales
                        confidence, predicted = torch.max(probs, 1) # Obtener clase con mayor probabilidad
                        
                        # Actualizar variables de estado (compartidas con MNIST_getNumber)
                        self.latest_digit = int(predicted.item())
                        self.latest_confidence = float(confidence.item())

                    # --- Visualización ---
                    # Dibujar recuadro verde y texto en la imagen de depuración
                    cv2.rectangle(color_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text = f"Num: {self.latest_digit} ({self.latest_confidence:.2f})"
                    cv2.putText(color_copy, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    # Mostrar lo que "ve" la red neuronal
                    # cv2.imshow("Input to DNN", inverted_roi)
                    pass
            else:
                # Si no se detecta panel, resetear valores
                self.latest_digit = -1
                self.latest_confidence = 0.0

            # Mostrar imagen original con detecciones
            # cv2.imshow("Camera360RGB", color_copy)
            # cv2.waitKey(1)

        except Exception as e:
            print(f"Error en compute: {e}")
            traceback.print_exc()

    # ---------------------------------------------------------
    # Detección de Cuadros (Visión Artificial Clásica)
    # ---------------------------------------------------------
    # Busca contornos rectangulares que parezcan paneles
    # ---------------------------------------------------------
    # Detección de Cuadros (Color Rojo)
    # ---------------------------------------------------------
    # Busca paneles con borde ROJO y fondo BLANCO
    def detect_frame(self, color):
        # Convertir a HSV para filtrar por color
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        
        # Rangos de color rojo (el rojo envuelve el 0/180)
        # Rango 1: 0-10
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        # Rango 2: 170-180
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
        # Operaciones morfológicas para limpiar ruido
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Encontrar contornos en la máscara roja
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        h, w, _ = color.shape
        best_cnt = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Filtrar ruido pequeño
            if area < 500: 
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = bw / float(bh)

            # Debe ser aproximadamente cuadrado (0.5 a 2.0)
            if 0.5 <= aspect_ratio <= 2.0:
                if area > max_area:
                    max_area = area
                    best_cnt = (x, y, bw, bh)

        # Devolver coordenadas del mejor candidato
        if best_cnt:
            x, y, bw, bh = best_cnt
            
            # ATENCION: El bounding box cubre el borde ROJO. 
            # El número está DENTRO. Hay que recortar el borde.
            # Asumimos que el borde tiene un grosor relativo.
            margin_x = int(bw * 0.15) # 15% de margen para quitar el borde rojo
            margin_y = int(bh * 0.15)
            
            x1 = max(0, x + margin_x)
            y1 = max(0, y + margin_y)
            x2 = min(w, x + bw - margin_x)
            y2 = min(h, y + bh - margin_y)
            
            if x2 > x1 and y2 > y1:
                return [x1, y1, x2, y2]

        return None

    def startup_check(self):
        print("Startup Check...")
        QTimer.singleShot(200, QApplication.instance().quit)

if __name__ == '__main__':
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    status = 0
    ic = None
    try:
        # Debug arguments
        print(f"DEBUG: sys.argv: {sys.argv}")
        
        # Load properties (Handling config file as arg if needed)
        initData = Ice.InitializationData()
        initData.properties = Ice.createProperties()
        if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
            initData.properties.load(sys.argv[1])
            
        ic = Ice.initialize(sys.argv, initData)
        properties = ic.getProperties()
        configData = {"Period": {"Compute": 100}} 
        try:
            period = properties.getPropertyAsIntWithDefault("CommonBehavior.Period", 100)
            configData["Period"]["Compute"] = period
        except: pass
        
        proxy_map = {}
        try:
            val_proxy = properties.getProperty("Proxies.Camera360RGB")
            print(f"DEBUG: Proxies.Camera360RGB -> '{val_proxy}'")
            camera_ep = val_proxy
            if camera_ep:
                # CAST IMPORTANTE: uncheckedCast para tener los métodos de la interfaz
                base_prx = ic.stringToProxy(camera_ep)
                proxy_map["Camera360RGB"] = ifaces.RoboCompCamera360RGB.Camera360RGBPrx.uncheckedCast(base_prx)
        except: pass

        
        worker = SpecificWorker(proxy_map, configData, startup_check=False)
        
        # --- Servidor ICE (Adapter) ---
        try:
            mnist_ep = properties.getProperty("Endpoints.MNIST")
            if mnist_ep:
                print(f"Creating Adapter 'MNIST' with endpoints: {mnist_ep}")
                adapter = ic.createObjectAdapterWithEndpoints("MNIST", mnist_ep)
                adapter.add(worker, ic.stringToIdentity("mnist"))
                adapter.activate()
                print("Adapter MNIST activated.")
            else:
                print("Warning: Property 'Endpoints.MNIST' not found. Server not started.")
        except Exception as e:
            print(f"Error starting ICE adapter: {e}")
            traceback.print_exc()

        app.exec()
    except:
        traceback.print_exc()
        status = 1
    if ic:
        try: ic.destroy()
        except: pass
    sys.exit(status)
