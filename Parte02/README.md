
# Part 2 - PSR

### Sumário
- Introdução ao [Python](https://www.python.org/) (Continuação)
    - Ciclos (`foor`, `while`)
    - Funções
    - Documentação com [pydoc](https://docs.python.org/3/library/pydoc.html)
    - Módulos
    - Tipos de Dados

# Introdução ao Python (continuação)

Diretivas gerais para uma melhor organização do trabalho nas aulas
- Cada aula deve estar numa pasta (diretório): Aula1, Aula2, etc.

- Os exercício 1 e 2 da aula de hoje devem ser feitos cada um no seu diretório (criar um para cada caso), mas a partir do Exercício 3 a metodologia consiste em manter-se no mesmo diretório e cada novo exercíco corresponderá a uma função nova criada, como se explica adiante.

- Múltiplas alíneas de um exercício fazem parte desse exercício.

Em alguns casos poderá ser útil duplicar um diretório existente (para criar novo exercício ou uma nova aula). Para criar o diretório e todo o seu conteúdo recursivamente para o novo pode-se usar o seguinte comando que copia toda a estrutura a começar em ex1 para o diretório ex2 que é criado se não existir.
```bash
$ cp -rp ex1 ex2
```

## Exercício 1
Copiar o exercício dos números perfeitos da [Parte 1](/Part01).  Introduzir comentários no início do ficheiro perfeito.py e antes de cada função para a descrever. Os comentários devem acrescentar informação relevante ao código.

```python
#!/usr/bin/env python3
# --------------------------------------------------
# A simple python script to check perfect numbers
# Eurico Pedrosa.
# PSR, September 2024.
# --------------------------------------------------

maximum_number = 100  # maximum number to test.

def getDividers(value):
    """
    Return a list of dividers for the number value
    :param value: the number to test
    :return: a list of dividers.
    """
    # your code here
    return []

def isPerfect(value):
    """
    Checks whether the number value is perfect.
    :param value: the number to test.
    :return: True or False
    """

    # your code here
    return False

def main():
    print("Starting to compute perfect numbers up to " + str(maximum_number))

    for i in range(0, maximum_number):
        if isPerfect(i):
            print('Number ' + str(i) + ' is perfect.')

if __name__ == "__main__":
    main()
```

## Exercício 2
Separar o ficheiro `primo.py` em dois ficheiros distintos.

1. `main.py`: inclui a função principal (main)
2. `my_functions.py`: contém a função de teste dos números.

Manter/reforçar os comentários apropriados.

Executar o script e gerar documentação com o pydoc.

##  Exercício 3
Criar uma cópia do exercício anterior e modificar o programa nos seguintes pontos:

1. `main.py`:
	Para interpretar um parâmetro de entrada (que será um limite
	máximo de até onde procurar números primos) e:
2. `my_functions.py`:
	Para o teste de divisores se fazer até à raiz quadrada do número
	e não até ao próprio número. Aumenta a eficiência do cálculo.

Usar um argumento de linha de comandos para definir o número máximo a testar (usar o package [argparse](https://docs.python.org/3/library/argparse.html)). No final deve-se executar com:

```bash
./main.py --max_numnber 1000
```

## Exercício 4: Ciclos "for", "while" e input do terminal
### 4 a)
Escrever uma função `printAllPreviousChars()` que não tem parâmetros e não tem retorno, mas que quando executada lê um caractere do teclado usando a função `readchar()` e imprime todos os caracteres desde o espaço `' '` até esse caractere, considerando a numeração de caracteres da tabela asccii.


O pacote `readchar` permite ler um caracter do terminal.
https://pypi.org/project/readchar/

Para instalar:

```bash
pip install --user readchar
```
Completar `main.py` o código que falta.

```python
def printAllCharsUpTo(stop_char):
    # <to complete>

def main():
    # <to complete>

if __name__ == '__main__':
    main()
```

### 4 b)
Adicionar a função `readAllUpTo(stop_char)` para ler caracteres de forma contínua e terminar quando chegar o caractere `'X'`.

### 4 c)
Criar a função `countNumbersUpto(stop_char)` para ler caracteres continuamente e terminar quando chegar o caractere `'X'`, e nessa altura indicar quantos caracteres são algarismos e quantos não são algarismos. Usar a função `isnumeric()`.

```python
def countNumbersUpTo(stop_char):
    total_numbers = 0
    total_others = 0
    while True:
        # add code here ...

    print('You entered ' + str(total_numbers) + ' numbers.')
    print('You entered ' + str(total_others) + ' others.')
```

## Exercício 5 - Tipos de dados em python
O python é uma linguagem denominada *dynamically typed*, por oposição às linguagens *typed* como o c em que é preciso indicar explicitamente o tipo de cada variável, ou outras em que as variáveis não têm tipo associado, como no caso do javascript.

Na prática, não é obrigatoriamente necessário indicar o tipo de uma variável e quando o tipo não é indicado, este é deduzido a partir de regras establecidas.

Mais informação:

https://www.tutorialspoint.com/What-are-the-differences-between-untyped-and-dynamically-typed-programming-languages

https://www.w3schools.com/python/python_datatypes.asp

### 5 a)
Usando como ponto de partida o exercício 4, alterar a função anterior para criar uma lista dos inputs realizados e processar essa lista (para calcular o número de digitos/outros) a posteriori.

```python
def countNumbersUpTo(stop_char):

    while True:
        # add code here to create a list of inputs

    total_numbers = 0
    total_others = 0
    for input in inputs:
        # process each input in the list

    print('You entered ' + str(total_numbers) + ' numbers.')
    print('You entered ' + str(total_others) + ' others.')
```

### 5 b)
Crie uma lista que contenha apenas os inputs numéricos que foram inseridos (pela ordem em que foram inseridos).

### 5 c)
Crie um dicionário apenas com os inputs other em que as chaves são a ordem dos inputs inseridos e o valor são os inputs.

### 5 d)
Reordene a lista da alínea 5 b) de modo a que esteja por ordem crescente do valor dos inputs.

### 5 e)
O python tem uma functionalidade chamada [list comprehension](https://www.pythonforbeginners.com/basics/list-comprehensions-in-python) que permite gerar a lista de números numa só linha de código. Veja o link e tente refazer a alínea **5 b)** usando *list comprehension*.
