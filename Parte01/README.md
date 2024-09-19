
# Parte 1 - PSR

### Sumário
- Introdução
    + Apresentação
    + Objetivos
    + Avaliação
- Ambiente de Desenvolvimento (Linux)
- Editores e IDEs
- Introdução ao git e ao github
- Metodologia
- Introdução ao [Python](https://www.python.org/)

## Introdução

Ver slides da apresentação da unidade curricular (UC) no elearning

## Ambiente de Desenvolvimento (Linux)

Esta UC pressupõe a utilização do Sistema Operativo Linux para a resolução dos exercícios e dos trabalhos práticos.

A distribuição recomendada é o [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/).
Existem diversas formas de usar o Ubuntu para quem tem outros sistemas operativos (Windows, MacOS). As mais interessantes são:

1. Usar uma imagem Ubuntu *live* - [Experimentar antes de usar](https://ubuntu.com/tutorials/try-ubuntu-before-you-install#1-getting-started).
2. Instalar Linux numa máquina virtual. [VirtualBox](https://www.virtualbox.org/) é um bom ponto de partida.
3. Instalar o Linux em [dual-boot](https://help.ubuntu.com/community/WindowsDualBoot) com o Windows (**recomendado**)

As duas primeiras soluções não interferem no disco nem no sistema operativo existente, mas são mais limitadas em termos de funcionalidades e desempenho. No caso da primeira, todo o trabalho que for feito se perde no fim da sessão se não for copiado para outro local. No caso da máquina virtual, vai ser preciso espaço em disco no ambinente Windows (ou MacOs) para criar a "imagem" do disco onde correrá o Linux em máquina virtual. É uma solução intermédia que funciona relativamente bem, mas como opera sobre o sistema operativo nativo, pode ter limitações de desempenho e ficará dependente da atividade desse sistema operativo (como as atualizações no Windows).

A terceira solução (dual-boot com o sistema operativo nativo) é a mais poderosa porque cada sistema operativo fica no seu próprio espaço e correm separadamente. Porém, é preciso repartir o disco que estaria todo atribuído ao sistema operativo nativo. O Linux oferece esta possibilidade durante a instalação e em geral o processo corre bem, mas há sempre o risco de perda de informação. Por isso, recomenda-se guardar toda a informação importante desenvolvida no sistema operativo original antes de fazer esta forma de instalação.

Mais informações podem ser obtidas em:
- https://ubuntu.com/tutorials
- https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

## Editores e IDEs

A ferramenta principal para criar e modificar ficheiros é o editor, muitas vezes integrado num ambiente de desenvolvimento (IDE). Há inúmeras opções desde simples editores (e.g. `gedit`, `kate`, `kwrite`) até ambientes de desenvolvimento muito sofisticados (e.g. `codeblocks`, `eclipse`, `vscode`, `pycharm`).

Além das propriedades fundamentais dos editores, hoje em dia são excelentes add-ons a "automated completion" (preenchimento automático de palavras e estruturas) , o "syntax highlight" (realce da sintaxe da linguagem), o "intellissense" (apresentação de todas as opções de preenchimento automático de campos e estruturas em variáveis, funções, etc.), ou a inserção automática de fragmentos de código padrão ("code snippets").

O editor com mais tradição por excelência é o `vim` (ou "vi improved") mas a sua utilização eficaz pode requerer anos de prática continuada e permite todas as facilidades indicadas acima, mas a sua configuração, por ser praticamente ilimitada, pode-se tornar complexa e, por isso, contraproducente em utilizadores iniciados.

**Recomenda-se o visual studio code**, que é gratuito. Pode ser instalado diretamente do gestor de aplicações do Ubuntu (Ubuntu Software) ou por [outras vias](https://askubuntu.com/questions/616075/how-do-i-install-visual-studio-code).

## Introdução ao git e ao github

Crie um repositório no github para esta UC.
Para isso terá de criar uma conta no github.
A sua conta pode ter vários repositórios com vários projetos.

Um exemplo: https://github.com/eupedrosa

Depois deve criar um repositório chamado `ua-psr-24-25` e descarregá-lo para o seu computador com o comando git clone:

```bash
git clone <url do repositório>
```

Para mais informações visite [Git and GitHub learning resources](https://docs.github.com/en/get-started/start-your-journey/git-and-github-learning-resources)

## Metodologia

Para melhor se desenvolver o trabalho nas aulas, deve-se seguir uma metodologia de organização de ficheiros em diretórios por aulas e por exercícios.

Dentro de cada aula, em especial nas primeiras, é também recomendado criar uma subpasta para cada exercício `ex1`, `ex2`, etc. Em certas aulas, ou aulas mais avançadas, os diversos exercícios serão feitos por acréscimo sucessivo sobre o código base dos exercícios anteriores; nessa altura serão dadas as instruções nesse sentido.

Os guiões para as aulas estarão a ser continuamente atualizados em:

https://github.com/eupedrosa/ua-psr-24-25

Recomenda-se que, sempre que possível, usem a versão online ou façam o update.

# Introdução ao Python

[Python](https://www.python.org/) é uma linguagem de programação simples, versátil e suportada por larga comunidade de desenvolvedores.
Possui uma enorme coleção de bibliotecas e *frameworks* que facilitam o desenvolvimento de projetos complexos.
Por exemplo, o [Django](https://www.djangoproject.com/) e [Flask](https://flask.palletsprojects.com/en/3.0.x/) para desenvolvimento web, [NumPy](https://numpy.org/) e [pandas](https://pandas.pydata.org/) para análise de dados, [TensorFlow](https://www.tensorflow.org/) e [PyTorch](https://pytorch.org/) para *machine learning* e muitos outros.

Essas características tornam o Python uma ferramenta ideal para o rápido desenvolvimento de sistemas robóticos. 

## Exercício 1

Desenvolva um programa que imprima no terminal a frase "Hello World".
Deve começar por editar o ficheiro `hello.py` (com o seu editor escolhido) e escrever
```python
# This is a comment !!

print("Hello World")
```

Execute o programa com o commando
```bash
$ python3 hello.py
```

Em alternativa, um ficheiro Python pode ser interpretado como se fosse um executável. Para tal, deve executar o seguinte passo (uma única vez):
```bash
$ chmod +x hello.py
```

Adicionalmente, `hello.py` tem que ter presente na sua primeira linha não vazia a seguinte instrução
```python
#!/usr/bin/env python3

# ^^^^^^^^^^^^^^^^^^^^
# This is called the Shebang Interpreter Directive
# https://linuxize.com/post/bash-shebang/

print("Hello World")
```

O programa pode agora ser executado com o commando
```bash
$ ./hello.py
```

## Exercício 2

Crie um programa designado `primos.py` que imprime no terminal números primos, um por linha, até um certo limite. Usar uma função auxiliar isPrime() que aceita um inteiro `n` e retorna `1` ou `0` conforme `n` for primo ou não.

```python
#!/usr/bin/env python3

maximum_number = 50

def isPrime(value):
    # add your code here

def main():
    print("Starting to compute prime numbers up to " + str(maximum_number))

    for i in range(0, maximum_number):
        if isPrime(i):
            print('Number ' + str(i) + ' is prime.')
        else:
            print('Number ' + str(i) + ' is not prime.')

if __name__ == "__main__":
    # __name__ is a special variable
    # Python automatically sets its value to “__main__” if the script is being run directly
    main()
```

Com a ajuda do programa `primos.py`, calcular quantos números primos inferiores a 10000 têm o algarismo 3.
```bash
./primos.py | grep "3" | wc -l
```
A resposta deve ser 561

## Exercício 3

Estender o exercício 2 de modo a:

1. Imprimir todos os divisores calculados para os números não primos;
2. Usar o pacote `colorama` para imprimir os números primos a verde;

NOTA: um pacote python pode ser instalado da seguinte forma:
```
pip install --user colorama
```
Se não tiver o `pip` instalado, correr `sudo apt install python3-pip`.
Se `pip` não existir experimente `pip3`.

## Exercício 4
Calcular números perfeitos (aqueles cuja soma dos divisores igualam o número) como por exemplo `6 = 3 + 2 + 1`.
Além do `main()` criar a função `isPerfect()`, que indica se o número é perfeito.

```python
#!/usr/bin/env python3

maximum_number = 100

def isPerfect(value):
    # add your code here
    return False

def main():
    print("Starting to compute perfect numbers up to " + str(maximum_number))

    for i in range(0, maximum_number):
        if isPerfect(i):
            print('Number ' + str(i) + ' is perfect.')


if __name__ == "__main__":
    main()
```