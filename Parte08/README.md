# Parte 8 - PSR

### Sumário

    Comunicações TCP-IP
    Introdução ao Robot Operating System (ROS)

Os sistemas robóticos são muito frequentemente sistemas complexos. Isto
leva a que o desenvolvimento de um programa que cubra todas as
funcionalidades necessárias seja uma tarefa longa e complicada.

Mais ainda, a utilização de um programa único cria dependências
desnecessárias e indesejáveis entre funcionalidades, mesmo em programas
organizados em multíplos ficheiros.

**A solução mais elegante para construir programas complexos é ... não o
fazer!**

De fato, o melhor é manter os programas desenvolvidos simples e focados
no menor número de funcionalidades possível. No entanto, isto implica
que estes programas sejam capazes de comunicar entre si.

Neste contexto, a comunicação entre programas torna-se um elemento
fundamental no desenvolvimento de sistemas robóticos avançados.

O Robot Operating System (ROS) é um conjunto de bibliotecas e
ferramentas *open source* concebidas para facilitar o desenvolvimento de
programas para robôs. Uma das grandes vantagens do ROS é a forma como
facilita a comunicação entre diferentes programas (nós).

# Exercício 0 - Instalação e Tutoriais de ROS

## Instalação

A versão do ROS que deve instalar é a **ROS Noetic Ninjemys** que requer a instalação do Ubuntu 20.04 ([guia de instalação](https://wiki.ros.org/noetic/Installation/Ubuntu)).

### Distribuição de linux mais recentes

Caso tenha no seu sistema uma distribuição diferente da recomendada (Ubuntu 20.04) pode utilizar o [Distrobox](https://distrobox.it/#installation).

Para a versão mais recente do Ubuntu, pode instalar o Distrobox da seguinte forma:

1. Instalar o podman. Podman é um substituto do docker que por omissão executa sem necessidade de `sudo`.
    ```bash
    sudo apt update && sudo apt install podman
    ```

2. Instalar o Distrobox
    ```bash
    wget -qO- https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo sh
    ```
3. Criar um `container` com Ubuntu 20.04
    ```bash
    distrobox create -i ubuntu:20.04 psr-ros-noetic
    ```
    Se tive uma placa gráfica da NVidia, deve adicionar a flag `--nvidia` quando estiver a criar o container:
    ```bash
    distrobox create -i ubuntu:20.04 --nvidia psr-ros-noetic
    ```
4. Entrar no container
    ```bash
    distrobox enter psr-ros-noetic
    ```
    A primeira vez que entrar no container, o Distrobox vai instalar 
    automaticamente o software necessário para o bom funcionamento do container.
5. Instalar sofware em falta
    Deve executar o comandando dentro do container:
    ```bash
    sudo apt install lsb-release
    ```

NOTA: Sempre que necessitar de utilizar o ROS, deve abrir um terminal e entrar dentro do container com `distrobox enter psr-ros-noetic.

### Ubuntu 20.04

Para instalar o ROS deve executar os seguintes passos:

```bash
sudo apt update && sudo apt install curl
```
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```bash
sudo apt update && sudo apt install ros-noetic-desktop-full python3-catkin-tools
```

Após a instalação pode executar os seguintes comandos para verificar uma
instalação correta:
```bash
source /opt/ros/noetic/setup.bash && roscore
```

Se tudo correu bem, um `rosmaster` deverá ser lançado. Utilize ctr+c para terminar o `rosmaster`.
Para mais informações leia o [guia de instalação](https://wiki.ros.org/noetic/Installation/Ubuntu).

## Tutoriais
Recomenda-se a realização de todos os tutoriais ROS na secção [1.1
begginer level](http://wiki.ros.org/ROS/Tutorials) antes da realização
dos exercícios desta aula. Pode eventualmente não fazer os tutoriais de
*c++*.

# Exercício 1 - Standalone TCP-IP Communication

Implemente este exemplo de [comunicação
TCP-IP](https://stackabuse.com/basic-socket-programming-in-python/) pura
(não baseado em ROS).

Neste exemplo, a comunicação ocorre entre um *programa servidor* e um
*programa cliente*. O cliente envia de dois em dois segundos informação
definida na lista *messages*. Note que a informação enviada é de vários
tipos (numérica, texto).

**server_test.py**
``` Python
#!/usr/bin/env python
# --------------------------------------------------
# Miguel Riem Oliveira.
# PSR, September 2020.
# Adapted from https://stackabuse.com/basic-socket-programming-in-python/
# --------------------------------------------------
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create TCP/IP socket
local_hostname = socket.gethostname()  # retrieve local hostname
local_fqdn = socket.getfqdn()  # get fully qualified hostname
ip_address = socket.gethostbyname(local_hostname)  # get the according IP address

# output hostname, domain name and IP address
print ("working on %s (%s) with %s" % (local_hostname, local_fqdn, ip_address))
server_address = (ip_address, 23456)  # bind the socket to the port 23456

print ('starting up on %s port %s' % server_address)
sock.bind(server_address)

# listen for incoming connections (server mode) with one connection at a time
sock.listen(1)

while True:
    print ('waiting for a connection')
    connection, client_address = sock.accept()  # wait for a connection

    try:  # show who connected to us
        print ('connection from', client_address)

        while True:  # receive the data in small chunks (64 bytes) and print it
            data = connection.recv(64)
            if data:
                print ("Data: %s" % data) # output received data
            else:
                print ("no more data.") # no more data -- quit the loop
                break
    finally:
        # Clean up the connection
        connection.close()
```

**client_test.py**
``` Python
#!/usr/bin/env python
# --------------------------------------------------
# Miguel Riem Oliveira.
# PSR, September 2020.
# Adapted from https://stackabuse.com/basic-socket-programming-in-python/
# -------------------------------------------------
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create TCP/IP socket
local_hostname = socket.gethostname()  # retrieve local hostname
local_fqdn = socket.getfqdn()  # get fully qualified hostname
ip_address = socket.gethostbyname(local_hostname)  # get the according IP address

server_address = (ip_address, 23456)  # bind the socket to the port 23456, and connect
sock.connect(server_address)
print ("connecting to %s (%s) with %s" % (local_hostname, local_fqdn, ip_address))

# define example data to be sent to the server
messages = [30, 'Robotics', 31, 14, 'Automation', 18]
for message in messages:
    print ('Sending message: ' + str(message))
    sock.sendall(message.encode())
    time.sleep(2)  # wait for two seconds

sock.close()  # close connection
```

Do ponto de vista de um programador de sistemas robóticos, este exemplo
acima tem várias desvantagens:

1.  **Complexidade**: A primeira é que o código tem muitos pormenores e
    detalhes (large code footprint), exigindo uma boa dose de atenção do
    programador, que devia estar preocupado com a aplicação robótica e
    não com a comunicação entre módulos.

2.  **Paradigma de comunicação**: Por outro lado, este programa está
    limitado à comunicação peer to peer entre um programa servidor e
    outro programa cliente. Este paradigma de comunicação, em que o
    programa cliente envia uma mensagem que tem obrigatóriamente de ser
    recebida, pode por vezes não ser o ideal.

3.  **Topologia de comunicação**: Este programa funciona numa
    comunicação do tipo peer to peer. Quer isto dizer que apenas existem
    dois programas a comunicar um com o outro. A inclusão de programas
    adicionais no ecosistema de comunicação não é imediata.

4.  **Serialização da informação transmitida** A informação que é
    enviada do cliente para o servidor tem de ser serializada. Quer isto
    dizer que a informação tem de ser colocada no formato de uma série
    de bytes consecutivos. Quando se usam estruturas de dados complexas
    (classes, dicionários, etc), isto implica um processo de conversão
    da informação (da estrutura de dados para um array de bytes, no
    cliente, e no sentido inverso do lado do servidor). Estes processos
    têm o nome de serialize / deserialize ou marshalling /
    unmarshalling, e podem ser complexos de implementar.

Obviamente que existem várias formas de resolver os problemas listados
acima. Por exemplo, para 2 e 3, existem bibliotecas que implementam
vários paradigmas e topologias de comunicação (e.g.
[zeromq](https://zeromq.org/)), e no caso 4 da serialização /
deserialização, também há soluçãos dedicadas a este problema (e.g.
[google protocol
buffers](https://developers.google.com/protocol-buffers)).

No entanto, o ponto aqui é que isto implicaria um grande esforço de
implementação e debugging focado nos problemas da comunicação. Recorde
que o objetivo principal era dividir um progama complexo em pequenos
programas que teriam de comunicar entre si. Esta solução só será válida
enquanto a comunicação entre módulos não impuser um grande aumento da
complexidade do sistema, caso contrário o propósito inicial de
simplificação é derrotado.

# Exercício 2 - Exemplo de serialização

Este exercício tem o objetivo de detalhar um processo de serialização /
deserialização. Partindo do Exercício 1, assuma que tem uma estrutura de
dados complexa que é uma instanciação de uma classe *Dog*, declarada num
ficheiro denominado *dog_lib.py*:

**dog_lib.py**
``` Python
from colorama import Fore, Style

class Dog:
    def __init__(self, name, color, age):
        self.name, self.color, self.age = name, color, age
        self.brothers = []  # no brothers for now

    def addBrother(self, name):
        self.brothers.append(name)

    def __str__(self):
        return 'name: ' + Fore.RED + str(self.name) + Fore.RESET + \
               ', age: ' + Fore.RED + str(self.age) + Fore.RESET +\
               ', color: ' + Fore.RED + str(self.color) + Fore.RESET +\
               ', brothers: ' + Fore.BLUE + str(self.brothers) + Style.RESET_ALL
```

**Do lado do cliente**, crie uma instância da class *Dog*, adicionando
alguns irmãos. e.g.:

``` Python
import dog_lib
dog = dog_lib.Dog(name='Toby', age=7, color='brown')  # instantiate a new dog
dog.addBrother('Lassie')
dog.addBrother('Boby')
print('CLIENT: my dog has ' + str(dog))
```

Depois envie o conteúdo desta classe numa mensagem para o servidor. Terá
de arranjar uma forma de colocar toda a informação contida na classe na
mensagem a enviar.

Depois, **do lado do servidor**, a mensagem deverá ser descodificada e
deve ser criada uma instância da classe *Dog* que espelhe a existente do
lado do cliente.

Imprima as instâncias nos dois programas para confirmar que são cópias
exatas.

# Exercício 3 - Publicação e subscrição em ROS

Crie um [novo pacote
ROS](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) com o nome
psr_aula8_ex3.

O pacote deve depender do `rospy` (aliás, todos os pacotes que
conteham programas em python devem depender do ropsy) e também do
[*std_msgs*](http://wiki.ros.org/std_msgs) e.g.:

    catkin_create_pkg psr_aula8_ex3 std_msgs rospy

Depois, adapte o [exemplo de publicação /
subscrição](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
de modo a que os dois programas possibilitem, com a inserção de
argumentos pela linha de comandos, definir o nome do tópico em que irão
escrever / ler. No caso do programa *publisher.py*, este deve ainda
permitir pelo mesmo mecanismo alterar o conteúdo da mensagem que envia
periódicamente bem como a frequência de envio.

Usando as novas funcionalidades implementadas. experimente lançar uma
constelação de nós para testar a flexibilidade do sistema de
comunicações do ROS. Por exemplo, lance um publicador do tópico
\"conversations\" e dois subscritores a este tópico. Depois lance um
outro publicador do tópico \"chat\" e apenas um subscritor.

Se ainda não o fez é altamente recomendável que instale e configure o
[terminator](http://www.linuxandubuntu.com/home/terminator-a-linux-terminal-emulator-with-multiple-terminals-in-one-window)
(ou similar, e.g.,
[tmux](https://linuxize.com/post/getting-started-with-tmux/)) de modo a
gerir mais facilmente a grande quantidade de programas a lançar.

Utilize o [rqt_graph](http://wiki.ros.org/rqt_graph) para visualizar em
tempo real o grafo de computação do sistema criado.

Veja este vídeo com um [exemplo](https://youtu.be/tzHbJkUsD-c).

# Exercício 4 - Serialização e deserialização em ROS

Apoiando-se no [tutorial para criaçao de mensagens em
ROS](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg),
faça a extensão do exercício 3 de modo a que a informação enviada seja a
mesma que no exercício 2 (class dog).

Sempre que quiser aproveitar código de um package ROS, deverá primeiro
criar o package novo e depois copiar o source code para o novo package.
Não pode pura e simplesmente copiar a pasta com o package, porque isso
não atualiza muita da informação que está nos ficheiros de configuração
*package.xml* e *CMakeLists.txt*. Crie um novo pacote ROS com o nome
*psr_aula8_ex4*, e copie para lá os ficheiros python do exercício
anterior.

Uma vez que a class dog não é standard, deverá criar uma mensagem custom
*Dog.msg* que contenha os mesmos campos da classe do exercício 2.

# Exercício 5 - Servidores e clientes em ROS

O paradigma de comunicação publicador / subscritor é bastante útil
quando o emissor da mensagem (o publicador) não tem interesse em receber
qualquer informação por parte do recetor (subscritor).

No entanto, há casos em que é interessante implementar uma comunicação
bidirecional, em que haja uma resposta a um pedido.

O ROS implementa este paradigma com uma tipologia denominada servidor
cliente. A iniciativa é tomada pelo cliente, que envia uma mensagem ao
servidor do serviço (a essa mensagem do cliente para o servidor chama-se
pedido), que deve ser respondida com uma mensagem do servidor para o
cliente (a esta mensagem chama-se resposta).

O objetivo deste exercício é fazer uma extensão do programa publicador
do exercício 4, de modo a que o programa *publisher.py* seja também o
servidor de um serviço que permite alterar o conteúdo da mensagem que
está a ser periodicamente publicada, nomeadamente a propriedade *name*
da classe *Dog*.

Começe por criar um novo serviço chamado *SetDogName.srv*, com o
seguinte conteúdo:

**SetDogName.srv**
``` srv
string new_name
---
bool result
```

O pedido é uma mensagem ROS que contem os campos acima do padrão
\"\-\--\", e a resposta do servidor contem os campos que estão abaixo do
mesmo padrão.

Depois implemente a funcionalidade do servidor do serviço *SetDogName*
no tópico *set_dog_name*.

Use os tutorials de [criação de servidores e clientes em
ROS](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29),
bem como o [tutorial de criação de
serviços](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg).
