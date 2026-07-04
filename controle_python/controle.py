import cv2
import numpy as np
import math
import serial
import time


ID_EIXO = 1
ID_POSICAO = 0
CAMERA_INDEX = 0

CAMERA_MATRIX = np.array([
    [879.76488,   0.00000, 313.06010],
    [  0.00000, 880.96040, 252.01975],
    [  0.00000,   0.00000,   1.00000]
], dtype=np.float32)

DIST_COEFFS = np.array([
    [0.05604, -0.03711, 0.00483, -0.00307, -0.84138]
], dtype=np.float32)


OFFSET_EIXO_3D = np.array([0.0, 28.0, 0.0], dtype=np.float32)  
OFFSET_POS_3D  = np.array([0.0, 28.0, 0.0], dtype=np.float32)  



PORTA = 'COM5'
BAUDRATE = 115200
TAXA_ENVIO_SEGUNDOS = 0.2
TAMANHO_ALVO_MM = 32.0
CRITERIO_ERRO = 0.05  
TOLERANCIA_MM = 3.0
TOLERANCIA_MM = TAMANHO_ALVO_MM * CRITERIO_ERRO
RAIO_AMARELO_MM = 30.0

def nada(x): pass

def create_aruco_detector():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, params)
        def detect(gray): return detector.detectMarkers(gray)
        return aruco, dictionary, params, detect
    else:
        def detect(gray): return aruco.detectMarkers(gray, dictionary, parameters=params)
        return aruco, dictionary, params, detect

def draw_label(img, text, xy, color):
    x, y = int(xy[0]), int(xy[1])
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

def main():
    aruco, dictionary, params, detect_markers = create_aruco_detector()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): raise RuntimeError("Não foi possível abrir a câmara.")

    JANELA_CAMERA = "Visual Servoing - Camera"
    JANELA_CONTROLES = "Painel de Controlo PID"
    JANELA_GRAFICO = "Telemetria PID - Tempo Real"
    
    cv2.namedWindow(JANELA_CAMERA, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(JANELA_CAMERA, 960, 720)
    cv2.namedWindow(JANELA_CONTROLES, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(JANELA_CONTROLES, 500, 150)
    cv2.namedWindow(JANELA_GRAFICO, cv2.WINDOW_AUTOSIZE)

    cv2.createTrackbar("Kp x100", JANELA_CONTROLES, 10, 500, nada)
    cv2.createTrackbar("Ki x100", JANELA_CONTROLES, 0, 200, nada)
    cv2.createTrackbar("Kd x100", JANELA_CONTROLES, 5, 200, nada)

    esp = serial.Serial()
    esp.port = PORTA; esp.baudrate = BAUDRATE; esp.timeout = 0.05
    esp.setDTR(True); esp.setRTS(True)
    serial_conectada = False
    try:
        esp.open(); time.sleep(2); esp.reset_input_buffer()
        print("Conectado ao ESP32!"); serial_conectada = True
    except Exception as e:
        print(f"Aviso: Sem comunicação serial ({e}).")

    ultimo_tempo_envio = time.time()
    tempo_frame_anterior = 0  
    movimento_liberado = False
    ir_para_home = False  
    
    memoria_pos_3d = None
    ultimo_tempo_visto = 0
    tara_x_mm = 0.0
    tara_y_mm = 0.0
    piscar_botao_tara = 0  


    erro_anterior_x = 0.0
    erro_anterior_y = 0.0
    integral_x = 0.0
    integral_y = 0.0
    ultimo_tempo_pid = time.time()
    MM_PARA_PASSOS = 40.0 

    frequencia_x = 0.0
    frequencia_y = 0.0
    dx_mm = 0.0
    dy_mm = 0.0
    dist_mm = 0.0

    LARGURA_GRAFICO = 600
    ALTURA_GRAFICO = 400
    historico_erro_x = list()  
    historico_erro_y = list()
    historico_vel_x = list()
    historico_vel_y = list()
    MAX_PONTOS = 200 

    filtro_dx = 0.0
    filtro_dy = 0.0
    ALFA_FILTRO = 0.3 
    primeira_leitura_filtro = True

    while True:
        ok, frame = cap.read()
        if not ok: break

        kp = cv2.getTrackbarPos("Kp x100", JANELA_CONTROLES) / 100.0
        ki = cv2.getTrackbarPos("Ki x100", JANELA_CONTROLES) / 100.0
        kd = cv2.getTrackbarPos("Kd x100", JANELA_CONTROLES) / 100.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detect_markers(gray)

        target_eixo_2d = None
        target_pos_2d = None
        pos_3d_eixo = None
        pos_3d_posicao = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_corners, marker_id in zip(corners, ids_flat):
                
                if marker_id == ID_EIXO:
                    tamanho_atual_mm = 42.0  
                elif marker_id == ID_POSICAO:
                    tamanho_atual_mm = 32.0 
                else:
                    continue 
                
                obj_points = np.array([
                    [-tamanho_atual_mm/2,  tamanho_atual_mm/2, 0],
                    [ tamanho_atual_mm/2,  tamanho_atual_mm/2, 0],
                    [ tamanho_atual_mm/2, -tamanho_atual_mm/2, 0],
                    [-tamanho_atual_mm/2, -tamanho_atual_mm/2, 0]
                ], dtype=np.float32)
                
                sucesso, rvec, tvec = cv2.solvePnP(
                    obj_points, marker_corners, CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if sucesso:
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, 20)
                    R, _ = cv2.Rodrigues(rvec)

                    if marker_id == ID_EIXO:
                        target_3d = np.dot(R, OFFSET_EIXO_3D) + tvec.flatten()
                        pos_3d_eixo = target_3d
                        p_2d, _ = cv2.projectPoints(target_3d.reshape(1,3), np.zeros(3), np.zeros(3), CAMERA_MATRIX, DIST_COEFFS)
                        target_eixo_2d = tuple(p_2d.reshape(2).astype(int))
                        cv2.circle(frame, target_eixo_2d, 5, (0, 255, 0), -1)

                    elif marker_id == ID_POSICAO:
                        target_3d = np.dot(R, OFFSET_POS_3D) + tvec.flatten()
                        pos_3d_posicao = target_3d
                        p_2d, _ = cv2.projectPoints(target_3d.reshape(1,3), np.zeros(3), np.zeros(3), CAMERA_MATRIX, DIST_COEFFS)
                        target_pos_2d = tuple(p_2d.reshape(2).astype(int))
                        cv2.circle(frame, target_pos_2d, 5, (255, 140, 0), -1)

        if pos_3d_posicao is not None:
            memoria_pos_3d = pos_3d_posicao.copy()
            ultimo_tempo_visto = time.time()
        else:
            tempo_cego = time.time() - ultimo_tempo_visto
            if memoria_pos_3d is not None and tempo_cego < 10000.0 and not ir_para_home:
                pos_3d_posicao = memoria_pos_3d
                p_2d, _ = cv2.projectPoints(pos_3d_posicao.reshape(1,3), np.zeros(3), np.zeros(3), CAMERA_MATRIX, DIST_COEFFS)
                target_pos_2d = tuple(p_2d.reshape(2).astype(int))
                cv2.circle(frame, target_pos_2d, 5, (100, 100, 100), -1)
                cv2.putText(frame, "MEMORIA", (target_pos_2d[0]+10, target_pos_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 2)

        if pos_3d_eixo is not None and pos_3d_posicao is not None:
            
            # 1. Posição Bruta vista pela câmera
            dx_bruto = (pos_3d_posicao[0] - pos_3d_eixo[0]) - tara_x_mm
            dy_bruto = (pos_3d_posicao[1] - pos_3d_eixo[1]) - tara_y_mm

            # 2. Aplicação do Filtro Exponencial (EMA)
            if primeira_leitura_filtro:
                filtro_dx = dx_bruto
                filtro_dy = dy_bruto
                primeira_leitura_filtro = False
            else:
                filtro_dx = (ALFA_FILTRO * dx_bruto) + ((1.0 - ALFA_FILTRO) * filtro_dx)
                filtro_dy = (ALFA_FILTRO * dy_bruto) + ((1.0 - ALFA_FILTRO) * filtro_dy)

            # 3. Repassa os valores limpos para o resto do sistema
            dx_mm = filtro_dx
            dy_mm = filtro_dy
            dist_mm = math.hypot(dx_mm, dy_mm)

            if target_eixo_2d is not None and target_pos_2d is not None:
                cv2.line(frame, target_eixo_2d, target_pos_2d, (255, 255, 0), 2)
                cv2.circle(frame, target_pos_2d, int(TOLERANCIA_MM * (650.0/pos_3d_posicao[2])), (0,255,0), 1)

            if dist_mm <= TOLERANCIA_MM:
                cor_vetor = (0, 255, 0); status_maquina = "ALINHADO (3D)"
            elif dist_mm <= RAIO_AMARELO_MM:
                cor_vetor = (0, 255, 255); status_maquina = "CORRIGINDO (MM)"
            else:
                cor_vetor = (0, 0, 255); status_maquina = "ERRO ALTO (3D)"

            # Cabeçalho IHM
            tempo_frame_atual = time.time()
            fps = 1 / (tempo_frame_atual - tempo_frame_anterior) if tempo_frame_anterior > 0 else 0
            tempo_frame_anterior = tempo_frame_atual

            cv2.rectangle(frame, (0, 0), (frame.shape[1], 45), (35, 35, 35), -1)
            cv2.line(frame, (0, 45), (frame.shape[1], 45), (100, 100, 100), 1)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            status_comunicacao = "SERIAL OK" if serial_conectada else "SERIAL OFFLINE"
            cor_fundo_serial = (0, 255, 0) if serial_conectada else (0, 0, 255)
            cor_texto_serial = (0, 0, 0) if serial_conectada else (255, 255, 255)
            cv2.rectangle(frame, (110, 10), (280, 35), cor_fundo_serial, -1)
            cv2.putText(frame, status_comunicacao, (120, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor_texto_serial, 2)

            cor_texto_estado = (255, 255, 255) if cor_vetor == (0, 0, 255) else (0, 0, 0)
            cv2.rectangle(frame, (295, 10), (590, 35), cor_vetor, -1)
            cv2.putText(frame, f"ST: {status_maquina}", (305, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor_texto_estado, 2)

            texto_pid = f"PID: [{kp:.2f}, {ki:.2f}, {kd:.2f}]"
            tamanho_texto_pid = cv2.getTextSize(texto_pid, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            pos_x_pid = frame.shape[1] - tamanho_texto_pid[0] - 15
            cv2.putText(frame, texto_pid, (pos_x_pid, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Rodapé
            cv2.rectangle(frame, (0, frame.shape[0] - 40), (550, frame.shape[0]), (20, 20, 20), -1)
            cv2.line(frame, (0, frame.shape[0] - 40), (550, frame.shape[0] - 40), (100, 100, 100), 1)
            draw_label(frame, f"Vetor Real: dX={dx_mm:.1f}mm | dY={dy_mm:.1f}mm | Dist={dist_mm:.2f}mm", (10, frame.shape[0] - 15), (255, 255, 255))

            tempo_atual_pid = time.time()
            dt = tempo_atual_pid - ultimo_tempo_pid
            ultimo_tempo_pid = tempo_atual_pid
            if dt <= 0: dt = 0.001 

            if not movimento_liberado or dist_mm <= TOLERANCIA_MM:
                integral_x = 0.0; integral_y = 0.0
                frequencia_x = 0.0; frequencia_y = 0.0
            else:
                erro_x = dx_mm
                erro_y = dy_mm

                integral_x += erro_x * dt
                integral_y += erro_y * dt
                LIMITE_INT = 300.0
                integral_x = max(min(integral_x, LIMITE_INT), -LIMITE_INT)
                integral_y = max(min(integral_y, LIMITE_INT), -LIMITE_INT)

                derivada_x = (erro_x - erro_anterior_x) / dt
                derivada_y = (erro_y - erro_anterior_y) / dt
                erro_anterior_x = erro_x; erro_anterior_y = erro_y

                saida_x_mm = (kp * erro_x) + (ki * integral_x) + (kd * derivada_x)
                saida_y_mm = (kp * erro_y) + (ki * integral_y) + (kd * derivada_y)

                frequencia_x = saida_x_mm * MM_PARA_PASSOS
                frequencia_y = saida_y_mm * MM_PARA_PASSOS

                LIMITADOR_FREQ = 2500.0 
                frequencia_x = max(min(frequencia_x, LIMITADOR_FREQ), -LIMITADOR_FREQ)
                frequencia_y = max(min(frequencia_y, LIMITADOR_FREQ), -LIMITADOR_FREQ)

            tempo_atual = time.time()
            if serial_conectada and (tempo_atual - ultimo_tempo_envio >= TAXA_ENVIO_SEGUNDOS):
                comando_formatado = f"{frequencia_x:.1f},{frequencia_y:.1f}\n"
                try:
                    esp.write(comando_formatado.encode('utf-8'))
                    esp.flush()
                except Exception: pass
                ultimo_tempo_envio = tempo_atual

        historico_erro_x.append(dx_mm)
        historico_erro_y.append(dy_mm)
        historico_vel_x.append(frequencia_x)
        historico_vel_y.append(frequencia_y)
        
        if len(historico_erro_x) > MAX_PONTOS:
            historico_erro_x.pop(0); historico_erro_y.pop(0)
            historico_vel_x.pop(0); historico_vel_y.pop(0)

        grafico = np.ones((ALTURA_GRAFICO, LARGURA_GRAFICO, 3), dtype=np.uint8) * 30 
        
        ZERO_Y = int(ALTURA_GRAFICO / 2) 
        cv2.line(grafico, (0, ZERO_Y), (LARGURA_GRAFICO, ZERO_Y), (100, 100, 100), 1, cv2.LINE_AA) 
        cv2.putText(grafico, "SETPOINT / TARGET (0.0mm)", (10, ZERO_Y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1, cv2.LINE_AA)


        if len(historico_erro_x) > 1:
            for i in range(1, len(historico_erro_x)):
                pt1_x = int((i - 1) * (LARGURA_GRAFICO / MAX_PONTOS))
                pt2_x = int(i * (LARGURA_GRAFICO / MAX_PONTOS))

                ESCALA_ERRO = 1.5    
                ESCALA_VEL = 0.05   



                cv2.line(grafico, (pt1_x, ZERO_Y - int(historico_erro_x[i-1] * ESCALA_ERRO)), 
                                  (pt2_x, ZERO_Y - int(historico_erro_x[i] * ESCALA_ERRO)), (0, 165, 255), 2, cv2.LINE_AA) # Laranja (Erro X)
                cv2.line(grafico, (pt1_x, ZERO_Y - int(historico_erro_y[i-1] * ESCALA_ERRO)), 
                                  (pt2_x, ZERO_Y - int(historico_erro_y[i] * ESCALA_ERRO)), (255, 0, 255), 2, cv2.LINE_AA) # Magenta (Erro Y)

                cv2.line(grafico, (pt1_x, ZERO_Y - int(historico_vel_x[i-1] * ESCALA_VEL)), 
                                  (pt2_x, ZERO_Y - int(historico_vel_x[i] * ESCALA_VEL)), (0, 255, 255), 1, cv2.LINE_4) # Amarelo (Vel X)
                cv2.line(grafico, (pt1_x, ZERO_Y - int(historico_vel_y[i-1] * ESCALA_VEL)), 
                                  (pt2_x, ZERO_Y - int(historico_vel_y[i] * ESCALA_VEL)), (0, 255, 0), 1, cv2.LINE_4) # Verde (Vel Y)

        cv2.rectangle(grafico, (0, 0), (LARGURA_GRAFICO, 40), (20, 20, 20), -1)
        cv2.putText(grafico, "Erro X (mm)", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 2, cv2.LINE_AA)
        cv2.putText(grafico, "Erro Y (mm)", (140, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(grafico, "Vel Motor X (Hz)", (265, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(grafico, "Vel Motor Y (Hz)", (415, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(JANELA_GRAFICO, grafico)

        if not movimento_liberado:
            cv2.rectangle(frame, (10, 60), (320, 90), (0, 140, 255), -1)
            cv2.putText(frame, "MOTORES TRAVADOS", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(frame, "Aperte ESPACO para armar", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1)
        else:
            cor_alerta = (0, 0, 255) if int(time.time() * 4) % 2 == 0 else (0, 0, 150)
            cv2.rectangle(frame, (10, 60), (320, 90), cor_alerta, -1)
            cv2.putText(frame, "CUIDADO: MOTORES ARMADOS", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cor_btn = (0, 255, 0) if piscar_botao_tara > 0 else (100, 100, 100)
        cv2.rectangle(frame, (frame.shape[1] - 160, 60), (frame.shape[1] - 10, 90), cor_btn, -1)
        cv2.putText(frame, "[C] ZERAR TARA", (frame.shape[1] - 150, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        if piscar_botao_tara > 0: piscar_botao_tara -= 1

        cv2.imshow(JANELA_CAMERA, frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            break
        elif key == ord(' '):  
            movimento_liberado = not movimento_liberado 
        elif key == ord('c'):  
            if pos_3d_eixo is not None and pos_3d_posicao is not None:
                tara_x_mm = pos_3d_posicao[0] - pos_3d_eixo[0]
                tara_y_mm = pos_3d_posicao[1] - pos_3d_eixo[1]
                piscar_botao_tara = 15 # Faz o botão piscar verde por 15 frames
                print(f"TARA REGISTRADA! Offset compensado: X={tara_x_mm:.2f}mm, Y={tara_y_mm:.2f}mm")

    if serial_conectada: esp.close()
    cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()