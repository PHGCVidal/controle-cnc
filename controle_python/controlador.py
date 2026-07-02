import serial
import time

PORTA = 'COM5'
BAUDRATE = 115200

esp = serial.Serial()
esp.port = PORTA
esp.baudrate = BAUDRATE
esp.timeout = 1
esp.setDTR(True)   # antes era False
esp.setRTS(True)   # antes era False

try:
    esp.open()
    time.sleep(2)
    esp.reset_input_buffer()
    print("Conectado! Escreva as coordenadas no formato X,Y (ex: 200,100)")
except Exception as e:
    print(f"Erro na porta: {e}")

while True:
    comando = input("\nDigite o setpoint (X,Y) ou 'sair': ")
    if comando.lower() == 'sair':
        break

    comando_formatado = f"{comando}\n"

    try:
        esp.write(comando_formatado.encode('utf-8'))
        esp.flush()
        time.sleep(0.5)

        while esp.in_waiting > 0:
            resposta = esp.readline().decode('utf-8', errors='ignore').strip()
            print(f"ESP32 Respondeu: {resposta}")

    except Exception as e:
        print(f"Erro de comunicação: {e}")