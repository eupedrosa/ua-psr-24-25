
# Parte 4 - PSR

### Sumário

    Avaliação - escrita de um programa para testes de typing.

## PSR Typing Test

Deve desenvolver um programa similar a um [typing test](https://www.typingtest.com/).

Em resumo, o programa deve mostrar ao utilizador letras geradas de forma
aleatória, ficando depois à espera que o utilizador insira a letra
indicada. Quando o utilizador insere uma letra, o programa passa para a
letra seguinte até que o desafio atinja uma tempo máximo predefinido.

Veja este vídeo exemplificativo do [programa em
execução](https://youtu.be/6tRTOd5vPH8). Atenção que o vídeo não mostra
a funcionalidade das palavras (ver abaixo).

Detalhadamente, as funcionalidades a desenvolver são as seguintes:

1.  criar um ficheiro main.py com a estrutura habitual

2.  o programa deve receber três argumentos de entrada que definem o
    modo de finalização do teste (tempo máximo, número de inputs máximo,
    e modo de caracter único ou de palavra).

    ```bash
    usage: main.py [-h] [-utm] [-mv MAX_VALUE]
    
    Definition of test mode
    
    optional arguments:
      -h, --help            show this help message and exit
    
      -utm, --use_time_mode
                            Max number of secs for time mode or maximum number of inputs for number of inputs mode.
    
      -mv MAX_VALUE, --max_value MAX_VALUE
                            Max number of seconds for time mode or maximum number of inputs for number of inputs mode.
    
      -uw, --use_words
                            Use word typing mode, instead of single character typing.
    ```

3.  o programa deve pedir ao utilizador que pressione uma tecla para
    começar o desafio

4.  depois de iniciado o desafio, o programa mostra uma letra minúscula
    ou uma palavra(gerada aleatoriamente) ao utilizador e aguarda o
    input do utilizador (um caracter apenas, sem o utilizador ter de
    pressionar o enter, ou no modo de palavra o número de caracteres da
    palavra mostrada)

5.  o funcionamento do teste deverá ter dois modos distintos: tempo
    máximo ou número de inputs máximo. O teste deve continuar até que
    seja ativada uma das seguintes condições de paragem:

    a.  No modo de tempo máximo, quando o tempo de teste decorrido for
        maior do que o tempo máximo definido

    b.  No modo de número de inputs máximo, quando o número de inputs
        dado pelo utilizador atingir o máximo predefinido

    c.  Quando o utilizador pressiona a tecla \"espaço\", o que leva à
        interrupção do teste (em qualquer dos modos)

6.  depois de cada input do utilizador, o programa deve imprimir uma
    mensagem indicando a tecla pressionada pelo utilizador e se foi ou
    não a correta

7.  para cada evento de input deve ser guardada a seguinte informação:
    letra mostrada (requested); letra recebida (received); tempo
    decorrido desde que é mostrada a letra até ao input do utilizador
    (duration).

    Defina um *namedtuple* chamado *Input* que contenha os três campos acima
    enumerados.

8.  depois de terminada a inserção de entradas, calcular várias
    estatisticas do teste, descritas na tabela seguinte:


    | Parâmetro                 | Descrição                 |
    | --- | --- |
    | test_duration             | Duração total do teste    |
    | test_start                | Data de início do teste   |
    | test_end                  | Data de término do teste  |
    | number_of_types           | Número de inputs          |
    | number_of_hits            | Número de inputs corretos |
    | accuracy                  | Precisão de inputs(hits/total) |
    | type_average_duration     | Duração média das respostas do utilizador |
    | type_hit_average_duration | Duração média das respostas corretas doutilizador |
    | type_miss_average_duration | Duração média das respostas incorretas do utilizador |
    
    Deve criar um dicionário python com todos estes campos, mais um campo
    adicional que é uma lista namedtuples do *Input*
    
    Um exemplo do dicionario criado (terá de ser adaptado para o caso das
    palavras):
    
    ```python
    {'accuracy': 0.0,
     'inputs': [Input(requested='v', received='s', duration=0.11265206336975098),
                Input(requested='w', received='d', duration=0.07883906364440918),
                Input(requested='d', received='a', duration=0.0720210075378418),
                Input(requested='a', received='s', duration=0.0968179702758789),
                Input(requested='b', received='d', duration=0.039067983627319336)],
     'number_of_hits': 0,
     'number_of_types': 5,
     'test_duration': 0.3997969627380371,
     'test_end': 'Thu Sep 10 16:36:20 2020',
     'test_start': 'Thu Sep 10 16:36:20 2020',
     'type_average_duration': 0.07987945793212418,
     'type_hit_average_duration': 0.0,
     'type_miss_average_duration': 0.07987945793212418}
    ```
    
    1.  o programa deve, antes de finalizar, imprimir o dicionário que foi
        criado
    
    Depois de criar o dicionario pode imprimi-lo com:
    
    ```python
    print(my_dict)
    ```
    
    No entanto, para ficar com uma impressão mais perceptível pode usar o
    package [prettyprint](https://docs.python.org/3/library/pprint.html):
    
    ```python
    from pprint import pprint
    pprint(my_dict)
    ```

## Avaliação

A avaliação será feita com base na existência e qualidade das funcionalidades acima descritas.
A organização e legibilidade do código também poderá ser tida em conta.
