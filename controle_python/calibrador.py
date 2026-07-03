import cv2
import numpy as np
import os

# =================================================================
# CONFIGURAÇÕES DO TABULEIRO (Preencha antes de rodar!)
# =================================================================
# O OpenCV conta as INTERSEÇÕES internas (quinas), não os quadrados.
# Um tabuleiro padrão de 10x7 quadrados tem 9x6 quinas internas.
CHESSBOARD_CORNERS = (10, 7) 

# Meça o lado de um quadradinho preto no papel impresso com um paquímetro
SQUARE_SIZE_MM = 13.3 

CAMERA_INDEX = 0
PASTA_IMAGENS = "calib_fotos"
# =================================================================

def main():
    if not os.path.exists(PASTA_IMAGENS):
        os.makedirs(PASTA_IMAGENS)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Não foi possível abrir a câmera.")

    print("\n--- MODO DE CAPTURA DE CALIBRAÇÃO ---")
    print("1. Aponte a câmera para o tabuleiro de xadrez impresso.")
    print("2. Pressione 's' para salvar uma foto. (Tire umas 15 a 20 fotos)")
    print("   -> Varie o ângulo, incline o tabuleiro, aproxime e afaste.")
    print("   -> Garanta que o tabuleiro inteiro apareça na tela.")
    print("3. Pressione 'c' quando terminar para CALCULAR as matrizes.")
    print("4. Pressione 'q' para sair sem fazer nada.\n")

    foto_count = 0

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Faz uma cópia para não salvar a imagem com os desenhos por cima
        frame_salvar = frame.copy() 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Tenta achar o tabuleiro em tempo real só para te dar feedback visual
        flags_ajuda = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        achou, corners = cv2.findChessboardCorners(gray, CHESSBOARD_CORNERS, flags_ajuda)
        
        if achou:
            cv2.drawChessboardCorners(frame, CHESSBOARD_CORNERS, corners, achou)
            cv2.putText(frame, "TABULEIRO DETECTADO - PODE SALVAR (s)", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Aguardando tabuleiro...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(frame, f"Fotos salvas: {foto_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Calibracao OpenCV", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') and achou:
            nome_arquivo = os.path.join(PASTA_IMAGENS, f"calib_{foto_count}.png")
            cv2.imwrite(nome_arquivo, frame_salvar)
            print(f"[{foto_count}] Foto salva: {nome_arquivo}")
            foto_count += 1
        elif key == ord('c'):
            print("\nIniciando cálculo... Isso pode levar alguns segundos.")
            break
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyAllWindows()

    if foto_count < 10:
        print("Aviso: É recomendado ter pelo menos 10 a 15 fotos para uma boa calibração!")

    # =================================================================
    # ETAPA 2: CÁLCULO MATEMÁTICO DAS MATRIZES
    # =================================================================
    # Prepara os pontos 3D no mundo real (0,0,0), (22.5,0,0), (45.0,0,0) ...
    objp = np.zeros((CHESSBOARD_CORNERS[0] * CHESSBOARD_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_CORNERS[0], 0:CHESSBOARD_CORNERS[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    objpoints = [] # Pontos 3D do mundo real
    imgpoints = [] # Pontos 2D na imagem

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    imagens_salvas = [os.path.join(PASTA_IMAGENS, f) for f in os.listdir(PASTA_IMAGENS) if f.endswith('.png')]

    print(f"Processando {len(imagens_salvas)} imagens...")

    for fname in imagens_salvas:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_CORNERS, None)

        if ret:
            objpoints.append(objp)
            # Refina a leitura do pixel para subpixel (aumenta precisão absurdamente)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

    if len(objpoints) > 0:
        # A Mágica Acontece Aqui:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        print("\n" + "="*50)
        print("CALIBRAÇÃO CONCLUÍDA COM SUCESSO!")
        print("Copie os blocos abaixo e cole no topo do seu código da CNC:")
        print("="*50 + "\n")

        print("CAMERA_MATRIX = np.array([")
        for row in camera_matrix:
            print(f"    [{row[0]:.5f}, {row[1]:.5f}, {row[2]:.5f}],")
        print("], dtype=np.float32)\n")

        print("DIST_COEFFS = np.array([")
        coeffs_str = ", ".join([f"{c:.5f}" for c in dist_coeffs[0]])
        print(f"    [{coeffs_str}]")
        print("], dtype=np.float32)\n")

        print(f"Erro de Reprojeção RMS: {ret:.4f} pixels")
        print("(Um RMS menor que 0.5 é excelente, menor que 1.0 é aceitável)")
    else:
        print("Falha: Nenhuma imagem válida com tabuleiro foi encontrada.")

if __name__ == "__main__":
    main()