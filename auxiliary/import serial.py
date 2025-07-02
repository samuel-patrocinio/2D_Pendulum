import serial
import csv
import time

# ====== CONFIGURAÇÕES ======
PORTA_SERIAL = 'COM4'       # Modifique para a porta correta (ex: 'COM4' no Windows ou '/dev/ttyUSB0' no Linux)
BAUD_RATE = 115200          # Mesmo baud rate do seu Serial.begin()
NOME_ARQUIVO = 'dados_serial_motor.csv'  # Nome do CSV de saída
TIMEOUT = 1                 # Tempo máximo de espera por linha
TEMPO_EXECUCAO = 30         # Tempo total de leitura em segundos (ou use Ctrl+C para parar manualmente)
# ===========================

def ler_serial_para_csv(porta, baudrate, arquivo_csv, tempo_execucao):
    ser = serial.Serial(porta, baudrate, timeout=TIMEOUT)
    print(f"Lendo da porta {porta}...")

    with open(arquivo_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        header_written = False
        inicio = time.time()

        try:
            while (time.time() - inicio) < tempo_execucao:
                linha = ser.readline().decode('utf-8').strip()
                if linha:
                    valores = linha.split(',')
                    if not header_written:
                        header = [f'valor_{i+1}' for i in range(len(valores))]
                        writer.writerow(header)
                        header_written = True
                    writer.writerow(valores)
                    print(valores)
        except KeyboardInterrupt:
            print("Interrompido pelo usuário.")
        finally:
            ser.close()
            print(f"Leitura encerrada. Dados salvos em {arquivo_csv}")

if __name__ == "__main__":
    ler_serial_para_csv(PORTA_SERIAL, BAUD_RATE, NOME_ARQUIVO, TEMPO_EXECUCAO)
