import serial
import threading
import time

# Configuration du port série
SERIAL_PORT = '/dev/USB_ODO'
BAUDRATE = 115200

stop_thread = False

hola = ("SET;START;1"
        "SET;PID;THETA;8;1;1")

def read_serial(ser):
    """Thread de lecture asynchrone du port série"""
    global stop_thread
    while not stop_thread:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    print(f"\n<<< {line}")
                    print("Commande >>> ", end="", flush=True)
            except Exception as e:
                print(f"\n[Erreur de lecture série] {e}")
        time.sleep(0.05)  # Évite d'occuper 100% du CPU

def send_command(ser, cmd):
    print(f">>> {cmd.strip()}")
    ser.write((cmd + '\n').encode())

def main():
    global stop_thread
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            time.sleep(2)  # Délai pour l'initialisation de l'appareil

            print("=== Interface de commande manuelle (asynchrone) ===")
            print("Tape une commande à envoyer, ou 'exit' pour quitter.\n")

            # Démarrer le thread de lecture série
            reader_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
            reader_thread.start()

            # Boucle principale de saisie
            while True:
                user_input = input("Commande >>> ").strip()
                
                if user_input.lower() in ('exit', 'quit'):
                    stop_thread = True
                    reader_thread.join(timeout=1)
                    print("Fermeture du programme.")
                    break
                elif user_input == "":
		    send_command(ser, hola)
                    continue
                else:
                    send_command(ser, user_input)

    except serial.SerialException as e:
        print(f"Erreur de port série : {e}")

if __name__ == "__main__":
    main()
