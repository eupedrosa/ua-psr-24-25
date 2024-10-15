
# Parte 6 - PSR

### Sumário
    Continuação do OpenCV.

# Exercício 1 - Anotações em imagens

Por vezes é importante desenhar informação em cima de uma imagem, seja
texto, figuras ou outros gráficos. O opencv disponibiliza várias funções
para o efeito.

## 1 a)

Carregue a imagem atlascar.png e desenhe um círculo no seu centro com a
função *cv2.circle*.

## 1 b)

Adicione o texto *PSR* à imagem a vermelho usando a funçao
`cv2.putText()`

## 1 c)

Crie um programa parecido com o paint. O programa deve ter um callback
que recolhe a posição do rato e quando o botão esquerdo for pressionado
o programa desenha pixeis de uma certa cor no ecrã. O ecrã deve ser uma
imagem de *600x400* inicializada toda branca.

O programa deve ainda deve escutar as teclas:

i.  tecla *r*, para mudar a cor a desenhar para vermelho

ii. tecla *g*, para mudar a cor a desenhar para verde

iii. tecla *b*, para mudar a cor a desenhar para azul

# Exercício 2 - Aquisição de video

O opencv também tem várias funções para lidar com vídeo. Ver [estes
tutoriais](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html).

> [!WARNING]
Naturalmente só conseguirá ter acesso às imagens de uma câmara se o seu
portátil tiver alguma com os drivers corretamente instalados.
Normalmente a utilização de máquinas virtuais não permite a utilização
de câmara. Pode também inserir uma câmara USB e tentar visualizá-la.
Pode usar o software cheese para tentar primeiro perceber se a [câmara
está a funcionar
corretamente](https://smallbusiness.chron.com/webcam-working-ubuntu-66873.html).

## 2 a)
Implemente um programa que faça a aquisição de uma imagem da câmara e
depois faça o seu display.

``` Python
#!/usr/bin/env python
import cv2

def main():
    # initial setup
    capture = cv2.VideoCapture(0)
    window_name = 'A5-Ex2'
    cv2.namedWindow(window_name,cv2.WINDOW_AUTOSIZE)

    _, image = capture.read()  # get an image from the camera

    # add code to show acquired image
    # add code to wait for a key press

if __name__ == '__main__':
    main()
```

## 2 b)

Adapte o exercício anterior de modo a implementar um programa que faça a
aquisição e display contínuos da imagem da câmara do seu portátil.

# Exercício 3 - Are you speaking?

Este exercício pretende que seja desenvolvido um programa mais complexo.

## 3 a) 

Pretende-se desenvolver uma aplicação em python opencv que seja capaz de
detetar caras num stream de video. A face detetada deve ser destacada na
imagem mostrada, ficando mais esverdeada.

Código exemplo para detetar uma cara e desenhar respetiva bounding box
```python
import cv2

face_classifier = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

def detect_bounding_box(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize=(40, 40))
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 4)
    return faces
```

## 3 b) 

Nas regiões da imagem que não estão incluídas pela cara detetada,
pretende-se detetar arestas e destacá-las avermelhando-as na imagem
mostrada.

> [!NOTE]
Explorem por exemplo as funções/classes `bitwise_not`, `dilate`, `add` e `Canny`.

## 3 c)

Finalmente, pretende-se que o programa seja capaz de perceber se a
pessoa está a falar ou não.

Esta classificação deve ser comunicada escrevendo texto na imagem
mostrada.

Exemplo: [How to Record Audio in Python: Automatically Detect Speech and Silence](https://dev.to/abhinowww/how-to-record-audio-in-python-automatically-detect-speech-and-silence-4951)

Requisitos:
```bash
sudo apt install libpython3-dev portaudio19-dev
pip install --user pyaudio webrtcvad
```

Podem ver um vídeo da aplicação [aqui](https://youtu.be/eMmuuiV5KGQ).
