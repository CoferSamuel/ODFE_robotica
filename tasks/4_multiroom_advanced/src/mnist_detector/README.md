# MNIST Detector

This component is a Python-based digit recognizer designed to identify apartment numbers for the `fullApartment` task. It uses a Convolutional Neural Network (CNN) trained on the MNIST dataset to perform real-time inference on images captured from a 360° camera.

## Features
- **CNN Inference:** Uses PyTorch for high-performance digit recognition.
- **Dynamic Optimization:** The component features a dual-mode execution cycle:
  - **IDLE Mode (500ms):** When no requests are pending, it runs at binary frequency to save CPU.
  - **FAST Mode (50ms):** When a `getNumber()` request is received, it switches to high frequency for 2 seconds.
- **Robustness:** Implements color-based frame detection (Red) to isolate the number plate before running the DNN.

## Dependencies

- **PySide6:** For the Qt event loop and timers.
- **OpenCV:** For image processing and frame detection.
- **PyTorch & Torchvision:** for DNN inference.
- **RoboComp:** The underlying middleware.

## Configuration parameters

The component configuration is located in `etc/config`. Key parameters include:

```ini
# Period of the main compute loop (default 500 when idle)
CommonBehavior.Period=100

# Proxy to the camera
Proxies.Camera360RGB=camera360rgb:tcp -h localhost -p 10000

# Endpoints for the MNIST service
Endpoints.MNIST=tcp -p 10010
```

## Starting the component

To run the component, follow these steps:

1. **Install dependencies:**
   ```bash
   pip install pyside6 opencv-python torch torchvision rich
   ```

2. **Run the component:**
   ```bash
   cd src/mnist_detector
   python src/specificworker.py etc/config
   ```

## Interface (IDS)

The component implements the `MNIST.idsl` interface:

```omni-idl
struct Digit {
    int val;
    float confidence;
};

interface MNIST {
    Digit getNumber();
};
```
