/*
marcaVisitados: Esta função realiza uma DFS (Depth-First Search) para marcar todos os vértices acessíveis a partir de um vértice inicial como visitados. Ela é usada para verificar a conectividade do grafo, ajudando a determinar se todos os vértices podem ser alcançados a partir de um ponto específico.

conexoNaoDir: Esta função verifica se um grafo não direcionado é conexo, ou seja, se há um caminho entre qualquer par de vértices. Para isso, utiliza a função marcaVisitados para marcar os vértices alcançáveis a partir do vértice 0 e, em seguida, verifica se todos os vértices foram visitados.

bipartido: A função determina se o grafo é bipartido, ou seja, se os vértices podem ser divididos em dois conjuntos disjuntos tal que não haja arestas entre vértices do mesmo conjunto. Ela utiliza uma BFS (Breadth-First Search) para tentar colorir o grafo com duas cores, verificando a validade da coloração.

euleriano: Esta função verifica se um grafo não direcionado é euleriano, o que significa que existe um ciclo que visita todas as arestas exatamente uma vez. Para isso, verifica se o grafo é conexo e se todos os vértices têm grau par.

bfs_ciclo: A função realiza uma BFS para detectar ciclos em um grafo não direcionado. Durante a BFS, se encontrar um vértice que já foi visitado e que não seja o pai do vértice atual, detecta-se a presença de um ciclo.

ciclo: Esta função verifica a presença de ciclos em todo o grafo, utilizando a função bfs_ciclo para cada componente conexo do grafo. Ela retorna true se algum ciclo for encontrado.

compConexo: A função calcula o número de componentes conexos em um grafo não direcionado. Ela utiliza a DFS para percorrer cada componente não visitado, incrementando um contador de componentes conexos.

compFortementeConexo: A função determina o número de componentes fortemente conexos em um grafo direcionado. Ela primeiro inverte as arestas do grafo, ordena os vértices pela ordem de término da DFS, e então realiza uma segunda DFS no grafo invertido.

marcaTempos: Esta função realiza uma DFS e registra os tempos de entrada e saída de cada vértice, que são usados para diversas análises em grafos, como a detecção de componentes fortemente conexos.

ordenaPar: Esta função auxilia na ordenação de pares de inteiros. Ela é usada para ordenar vértices com base em seus tempos de saída na DFS, uma operação necessária para a identificação de componentes fortemente conexos.

artiucladosDFS: A função auxilia na identificação de pontos de articulação em um grafo. Pontos de articulação são vértices cuja remoção aumenta o número de componentes conexos do grafo. A função realiza uma DFS modificada para calcular os valores de low e discovery e determinar os pontos de articulação.

articulados: Esta função identifica todos os pontos de articulação em um grafo, utilizando a função artiucladosDFS para realizar a DFS a partir de cada vértice. Ela retorna uma lista contendo todos os pontos de articulação encontrados.

dfsPontes: A função realiza uma DFS para detectar pontes em um grafo. Uma ponte é uma aresta cuja remoção aumenta o número de componentes conexos do grafo. A função atualiza os valores de low e tempoEntrada para identificar essas arestas críticas.

pontes: Esta função encontra todas as pontes em um grafo, utilizando a função dfsPontes para percorrer cada componente do grafo. Ela retorna o número total de pontes encontradas.

final: A função verifica se todos os vizinhos de um determinado vértice foram visitados durante a execução de uma DFS. É usada para controlar a continuidade da busca.

dfs: Esta função realiza uma DFS no grafo, retornando uma lista de arestas percorridas. Ela serve como uma implementação básica de DFS, útil para percorrer o grafo e coletar informações sobre as arestas.

bfs: A função realiza uma BFS no grafo, retornando uma lista de arestas percorridas. Semelhante à dfs, mas utiliza uma busca em largura para explorar o grafo.

prim: Esta função implementa o algoritmo de Prim para encontrar a árvore geradora mínima (MST) de um grafo ponderado. Ela usa uma fila de prioridade para selecionar as arestas de menor peso que conectam novos vértices à árvore em construção.

dfsPontes: Esta função executa uma DFS (Depth-First Search) para detectar pontes em um grafo. Uma ponte é uma aresta cuja remoção aumenta o número de componentes conectados do grafo. A função utiliza dois vetores, menorTempo e tempoEntrada, para acompanhar o menor tempo de visitação e o tempo de entrada de cada vértice durante a DFS. Se o menor tempo de visitação do vizinho for maior que o tempo de entrada do vértice atual, a aresta é uma ponte. O número total de pontes é incrementado durante o processo.

pontes: A função pontes invoca a função dfsPontes para cada vértice do grafo e contabiliza o número total de pontes. Inicialmente, ela cria vetores de controle para marcar os vértices visitados, armazenar o menor tempo de visitação e o tempo de entrada de cada vértice. Ao final, ela retorna o número total de pontes detectadas no grafo, que é útil para análises de conectividade e robustez do grafo.

djikstra: Esta função implementa o algoritmo de Dijkstra para encontrar o caminho mais curto de um vértice de origem a todos os outros vértices em um grafo ponderado. Utiliza uma fila de prioridade para selecionar o vértice com a menor distância acumulada e, em seguida, atualiza as distâncias dos vértices adjacentes. O algoritmo continua até que todos os vértices tenham sido processados, garantindo que a menor distância de cada vértice ao ponto de origem seja calculada.

fechoTrans: A função fechoTrans calcula o fecho transitivo de um grafo, especificamente para o vértice 0. O fecho transitivo de um vértice é o conjunto de todos os vértices que podem ser alcançados a partir dele. A função utiliza uma DFS para explorar todos os caminhos possíveis a partir do vértice 0 e retorna uma lista dos vértices que fazem parte desse fecho.

ordemTopologica: Esta função realiza a ordenação topológica de um grafo direcionado acíclico (DAG). Utiliza uma DFS para visitar os vértices e empilhá-los na ordem inversa de seus tempos de saída. A ordenação resultante garante que, para cada aresta u → v, o vértice u apareça antes do vértice v na lista final, o que é essencial em muitos algoritmos de ordenação e programação.

fluxoMaximo: Implementa o algoritmo de Ford-Fulkerson para calcular o fluxo máximo em um grafo de fluxo. O fluxo máximo é a maior quantidade de fluxo que pode ser enviado de uma fonte a um destino em uma rede. A função utiliza uma BFS para encontrar caminhos aumentadores no grafo residual e ajusta as capacidades das arestas até que nenhum caminho aumentador possa ser encontrado. O resultado final é o valor do fluxo máximo que pode ser alcançado.
*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <limits.h>
#include <sstream>
#include <bits/stdc++.h>

using namespace std;

vector<int> articulados(vector<vector<pair<int, pair<int, int>>>>& adj, int qtdVertices);
vector<int> bfs(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices);
bool bipartido(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices);
bool ciclo(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices);
bool bfs_ciclo(vector<vector<pair<int, pair<int, int>>>> lista_adj, int origem, vector<int>& pais);
int compConexo(vector<vector<pair<int, pair<int, int>>>> listaAdj, int qtdVertices);
int compFortementeConexo(vector<vector<pair<int, pair<int, int>>>> listaAdj,int qtdVertices);
vector<int> marcaTempos(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices);
bool ordenaPar(pair<int,int> a,pair<int,int> b);
bool conexoNaoDir(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int numVertices);
bool final(vector<int> cor, vector<pair<int, pair<int, int>>>& vizinhos);
vector<int> dfs(vector<vector<pair<int, pair<int, int>>>>& lista_adj, int qtdVertices);
int djikstra(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices);
bool euleriano (vector<vector<pair<int, pair<int, int>>>>& listaAdj, int numVertice);
vector <int> fechoTrans(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices);
void marcaVisitados(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int vertice, vector<bool>& visitados);
int pontes(vector<vector<pair<int, pair<int, int>>>>& listaAdj);
int prim(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices, int& pesoTotal);
vector<int> ordenacao_topologica(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices);
int fluxoMaximo(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices);


#define BRANCO 0 //Ainda não visitado
#define PRETO 1  //Pintado de preto
#define CINZA 2 //Pintado de vermelho
#define INF numeric_limits<int>::max()

void marcaVisitados(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int vertice, vector<bool>& visitados){
    visitados[vertice] = true;
    for (auto& elemento : listaAdj[vertice])
    {
        int vizinho = elemento.second.first;
        if (!visitados[vizinho])
        {
            marcaVisitados(listaAdj, vizinho, visitados);
        }
    }
}

bool conexoNaoDir(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int numVertices){
    vector<bool> visitados(numVertices, false);
    marcaVisitados(listaAdj, 0, visitados);
    for (bool x : visitados)
    {
        if(x == false) return false;
    }
    return true;
}


bool bipartido(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices){
    vector<int> cor(qtdVertices, BRANCO);
    for (int i = 0; i < qtdVertices; i++)
    {
        if(cor[i] == BRANCO){ 
            cor[i] = PRETO;
            queue<int> fila;
            fila.push(i);

            while (!fila.empty())
            {
                int verticeAtual = fila.front();
                fila.pop();
                for(auto& vizinhos : listaAdj[verticeAtual]){
                    int vizinhoAtual = vizinhos.second.first;
                    if(cor[vizinhoAtual] == BRANCO){
                        if(cor[verticeAtual] == PRETO)cor[vizinhoAtual] = CINZA;
                        else cor[vizinhoAtual] = PRETO;
                        fila.push(vizinhoAtual);
                    }else if(cor[vizinhoAtual] == cor[verticeAtual]) return false;
                }
            }
        }
    }
    return true;
}

bool euleriano (vector<vector<pair<int, pair<int, int>>>>& listaAdj, int numVertice){

    if(!conexoNaoDir(listaAdj, numVertice)) return false;    

    for(auto& relacoes : listaAdj){
        int contVertice = relacoes.size();
        if (contVertice % 2 != 0)
        {
            return false;
        }
    }

    return true;
}

bool bfs_ciclo(vector<vector<pair<int, pair<int, int>>>> lista_adj, int origem, vector<int>& pais) {
    vector<bool> visitados(lista_adj.size(), false);
    queue<int> fila;

    fila.push(origem);
    visitados[origem] = true;
    pais[origem] = -1; // O vértice de origem não tem pai

    while (!fila.empty()) {
        int u = fila.front();
        fila.pop();

        for (auto& vizinho : lista_adj[u]) {
            int v = vizinho.second.first;

            if (!visitados[v]) {
                fila.push(v);
                visitados[v] = true;
                pais[v] = u;
            } else if (v != pais[u]) {
                return true; // Encontrou um ciclo
            }
        }
    }

    return false;
}

bool ciclo(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices) {
    vector<int> pais(qtdVertices, -1);

    for (int i = 0; i < qtdVertices; i++) {
        if (pais[i] == -1) {
            if (bfs_ciclo(lista_adj, i, pais)) {
                return true; // Ciclo encontrado
            }
        }
    }

    return false; // Nenhum ciclo encontrado
}

int compConexo(vector<vector<pair<int, pair<int, int>>>> listaAdj, int qtdVertices){
    vector<bool> visitados(qtdVertices, false);
    int qtdComponentes = 0;

    for (int i = 0; i < qtdVertices; i++)
    {
        if (!visitados[i])
        {
            marcaVisitados(listaAdj, i, visitados);
            qtdComponentes++;
        }   
    }

    return qtdComponentes;
}

int compFortementeConexo(vector<vector<pair<int, pair<int, int>>>> listaAdj, int qtdVertices){
    int componentes = 0;
    vector<vector<pair<int, pair<int, int>>>> inverso(qtdVertices);
    for(int i = 0; i < qtdVertices; i++){
        for(auto aresta: listaAdj[i]){
            int origem = i;
            int id = aresta.first;
            int destino = aresta.second.first;
            int peso = aresta.second.second;
            inverso[destino].push_back(make_pair(id, make_pair(origem, peso)));
        }
    }
    vector<int> out(qtdVertices);

    out = marcaTempos(listaAdj, qtdVertices);
    vector<pair<int,int>> outv(qtdVertices);
    sort(outv.begin(),outv.end(),ordenaPar);

    vector<int> cor(qtdVertices, BRANCO);

    for(int i = 0; i < qtdVertices; i++){
        if(cor[i] == BRANCO){
            vector<int> pilha;
            componentes++;
            pilha.push_back(i);
            while(!pilha.empty()) {
                int u = pilha.back();
                if(!final(cor, inverso[u])) {
                    for(auto& vizinho : inverso[u]) {
                        int v = vizinho.second.first;
                        if(cor[v] == BRANCO) {
                            pilha.push_back(v);
                            cor[v] = CINZA;
                            break;
                        }
                    }
                }else {
                    cor[u] = PRETO;
                    pilha.pop_back();
                }
            }
        }
    }
    return componentes;
}

vector<int> marcaTempos(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices){
    vector<int> entrada(qtdVertices,0);
    vector<int> saida(qtdVertices,0);
    vector<int> cor(qtdVertices, BRANCO);
    vector<int> pilha;

    int origem = 0;
    int passos = 1;

    pilha.push_back(origem);
    cor[origem] = CINZA;

    while(!pilha.empty()) {
        int u = pilha.back();
        if(entrada[u]==0){
            entrada[u] = passos; 
            passos++;
        }
        if(!final(cor, lista_adj[u])) {
            for(auto& vizinho : lista_adj[u]) {
                int v = vizinho.second.first;
                if(cor[v] == BRANCO) {
                    pilha.push_back(v);
                    cor[v] = CINZA;
                    break;
                }
            }
        } else {
            cor[u] = PRETO;
            saida[u] = passos;
            passos++;
            pilha.pop_back();
        }
    }
    return saida;
}

bool ordenaPar(pair<int,int> a,pair<int,int> b){
       return a.second>b.second;
}

void artiucladosDFS(int u, vector<vector<pair<int, pair<int, int>>>>& adj, int* discovery, int* low, int* parent, vector<bool>& articulation, int* cor, int tempo) {
    cor[u] = CINZA;
    discovery[u] = low[u] = ++tempo;
    int filhos = 0;

    for (const auto& edge : adj[u]) {  // Percorre as arestas conectadas ao vértice u
        int v = edge.second.first; // Obtém o índice do vértice de destino a partir do segundo par

        if (cor[v] == BRANCO) { // v ainda não foi visitado
            filhos++;
            parent[v] = u;
            artiucladosDFS(v, adj, discovery, low, parent, articulation, cor, tempo);

            // Verifica se a subárvore de v pode se conectar a um ancestral de u
            low[u] = min(low[u], low[v]);

            // Condição 1: U é raiz e tem mais de um filho
            if (parent[u] == -1 && filhos > 1) {
                articulation[u] = true;
            }

            // Condição 2: U não é raiz e low[v] >= discovery[u]
            if (parent[u] != -1 && low[v] >= discovery[u]) {
                articulation[u] = true;
            }
        } else if (v != parent[u]) { // v já foi visitado e não é o pai de u
            low[u] = min(low[u], discovery[v]);
        }
    }

    cor[u] = CINZA;
}

vector<int> articulados(vector<vector<pair<int, pair<int, int>>>>& adj, int qtdVertices) {
    int* discovery = new int[qtdVertices];
    int* low = new int[qtdVertices];
    int* parent = new int[qtdVertices];
    int* cor = new int[qtdVertices];
    vector<bool> articulation(qtdVertices, false);
    int tempo = 0;

    for (int i = 0; i < qtdVertices; i++) {
        cor[i] = BRANCO;
        parent[i] = -1;
    }

    for (int u = 0; u < qtdVertices; u++) {
        if (cor[u] == BRANCO) {
            artiucladosDFS(u, adj, discovery, low, parent, articulation, cor, tempo);
        }
    }

    vector<int> pontosDeArticulacao;
    for (int i = 0; i < qtdVertices; i++) {
        if (articulation[i]) {
            pontosDeArticulacao.push_back(i);
        }
    }

    // Liberar memória
    delete[] discovery;
    delete[] low;
    delete[] parent;
    delete[] cor;

    return pontosDeArticulacao;
}

void dfsPontes(int vertice, int pai, vector<vector<pair<int, pair<int, int>>>>& listaAdj, vector<int>& visitado, vector<int>& menorTempo, vector<int>& tempoEntrada, int& tempo, int& qtdPontes) {
    visitado[vertice] = true;
    menorTempo[vertice] = tempoEntrada[vertice] = tempo++;
    
    for (auto& elemento : listaAdj[vertice]) {
        int vizinho = elemento.second.first;
        if (vizinho == pai) continue; 
        
        if (!visitado[vizinho]) {
            dfsPontes(vizinho, vertice, listaAdj, visitado, menorTempo, tempoEntrada, tempo, qtdPontes);
            
            
            if (menorTempo[vizinho] > tempoEntrada[vertice]) {
                qtdPontes++; 
            }
            
            menorTempo[vertice] = min(menorTempo[vertice], menorTempo[vizinho]);
        } else {
            menorTempo[vertice] = min(menorTempo[vertice], tempoEntrada[vizinho]);
        }
    }
}

int pontes(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices) {
    vector<int> visitado(qtdVertices, false);
    vector<int> menorTempo(qtdVertices, -1);
    vector<int> tempoEntrada(qtdVertices, -1);
    int tempo = 0;
    int qtdPontes = 0;

    // Executa a DFS a partir de todos os vértices para garantir que todos os componentes sejam visitados
    for (int i = 0; i < qtdVertices; ++i) {
        if (!visitado[i]) {
            dfsPontes(i, -1, listaAdj, visitado, menorTempo, tempoEntrada, tempo, qtdPontes);
        }
    }

    return qtdPontes;
}

bool final(vector<int> cor, vector<pair<int, pair<int, int>>>& vizinhos) {
    for(auto& vizinho : vizinhos){
        int v = vizinho.second.first;
        if(cor[v] == BRANCO){
            return false;
        }
    }
    return true;
}

vector<int> dfs(vector<vector<pair<int, pair<int, int>>>>& lista_adj, int qtdVertices) {
    vector<int> cor(qtdVertices, BRANCO);
    vector<int> arestas;
    vector<int> pilha;

    int origem = 0;

    pilha.push_back(origem);
    cor[origem] = CINZA;

    while(!pilha.empty()) {
        int u = pilha.back();
        if(!final(cor, lista_adj[u])) {
            for(auto& vizinho : lista_adj[u]) {
                int id = vizinho.first;
                int v = vizinho.second.first;
                if(cor[v] == BRANCO) {
                    pilha.push_back(v);
                    cor[v] = CINZA;
                    arestas.push_back(id);
                    break;
                }
            }
        } else {
            cor[u] = PRETO;
            pilha.pop_back();
        }
    }

    return arestas;
}

vector<int> bfs(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices){
     vector<int> cor(qtdVertices, BRANCO);

    int origem = 0;
    queue<int> fila;
    vector<int> arestas;
    fila.push(origem);
    cor[origem] = CINZA;

    while(!fila.empty()){
        int u = fila.front();
        fila.pop();
        for(auto& vizinho : lista_adj[u]) {
            int id = vizinho.first;
            int v = vizinho.second.first;

            if(cor[v] == BRANCO){
                cor[v] = CINZA;
                arestas.push_back(id);
                fila.push(v);
            }
        }
        cor[u] = PRETO;
    }

    // Exibindo o vetor de arestas que foram percorridas na BFS
    return arestas;
}

int prim(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices, int& pesoTotal){
    vector<int> key(qtdVertices, INT_MAX);
    vector<bool> inMST(qtdVertices, false);
    vector<int> pai(qtdVertices, -1);

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> min_heap;

    for (int i = 0; i < qtdVertices; i++)
    {
        if(!inMST[i]){
            key[i] = 0;
            min_heap.push({0,i});

            while (!min_heap.empty())
            {
                int x = min_heap.top().second;
                min_heap.pop();

                if (inMST[x]) continue;

                inMST[x] = true;

                for (auto& relacao : listaAdj[x]) {
                    int destino = relacao.second.first;
                    int peso = relacao.second.second;
                    if (peso && !inMST[destino] && peso < key[destino])
                    {
                        key[destino] = peso;
                        min_heap.push({key[destino], destino});
                        pai[destino] = x;
                    }
                }
            }
        }
    }

    for (int i = 0; i < qtdVertices; i++)
    {
        if (pai[i] != -1)
        {
            pesoTotal += key[i];
        }
    }

    int soma_pesos = 0;
    int vertice = qtdVertices - 1;
    while (vertice != 0)
    {   
        int pai_vertice = pai[vertice];
        for (auto& relacao : listaAdj[pai_vertice])
        {
            if (relacao.second.first == vertice)
            {
                soma_pesos += relacao.second.second;
                break;
            }
        }
        vertice = pai_vertice;
    }
    return soma_pesos;

};

vector<int> ordenacao_topologica(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices) {
    vector<int> cor(qtdVertices, BRANCO);
    vector<int> ordem;
    for(int i = 0; i < qtdVertices; i++){
        if(cor[i] == BRANCO){
            vector<int> pilha;
            pilha.push_back(i);
            while(!pilha.empty()) {
                int u = pilha.back();
                if(!final(cor, lista_adj[u])) {
                    for(auto& vizinho : lista_adj[u]) {
                        int v = vizinho.second.first;
                        if(cor[v] == BRANCO) {
                            pilha.push_back(v);
                            cor[v] = CINZA;
                            break;
                        }
                    }
                }else {
                    cor[u] = PRETO;
                    ordem.insert(ordem.begin(), u);
                    pilha.pop_back();
                }
            }
        }
    }
    return ordem;
}

int djikstra(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices){
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> min_heap;
    vector<int> distancia(qtdVertices, INT_MAX);

    min_heap.push(make_pair(0,0));
    distancia[0] = 0;

    while (!min_heap.empty())
    {
        int u = min_heap.top().second;
        min_heap.pop();

        for(auto& vizinho : listaAdj[u]){
            int v = vizinho.second.first;
            int peso = vizinho.second.second;

            if(distancia[u] + peso < distancia[v]){
                distancia[v] = distancia[u] + peso;
                min_heap.push(make_pair(distancia[v], v));
            }
        }
    }
    return distancia[qtdVertices-1];

}

//14 - COLOCAR
int fluxoMaximo(vector<vector<pair<int, pair<int, int>>>> lista_adj, int qtdVertices) {
    // Inicializa a matriz de fluxo
    vector<vector<int>> fluxo(qtdVertices, vector<int>(qtdVertices, 0));
    int fluxo_maximo = 0;

    vector<int> pai(qtdVertices); // Vetor para armazenar o caminho

    // Enquanto houver caminho aumentador
    while (true) {
        vector<int> cor(qtdVertices, BRANCO);
        queue<int> fila;
        fila.push(0); // Fonte
        cor[0] = CINZA;

        bool caminhoEncontrado = false;

        while (!fila.empty() && !caminhoEncontrado) {
            int u = fila.front();
            fila.pop();

            for (auto vizinho : lista_adj[u]) {
                int v = vizinho.second.first;
                int capacidadeResidual = vizinho.second.second - fluxo[u][v]; // Capacidade - fluxo

                if (cor[v] == BRANCO && capacidadeResidual > 0) {
                    cor[v] = CINZA;
                    pai[v] = u;
                    if (v == qtdVertices - 1) {
                        caminhoEncontrado = true; // Caminho aumentador encontrado
                        break;
                    }
                    fila.push(v);
                }
            }
            cor[u] = PRETO;
        }

        if (!caminhoEncontrado) break; // Nenhum caminho aumentador encontrado

        // Encontra a capacidade mínima no caminho
        int gargalo = numeric_limits<int>::max();
        for (int v = qtdVertices - 1; v != 0; v = pai[v]) {
            int u = pai[v];
            int capacidadeResidual = 0;
            for (const auto& aresta : lista_adj[u]) {
                if (aresta.second.first == v) {
                    capacidadeResidual = aresta.second.second - fluxo[u][v];
                    break;
                }
            }
            gargalo = min(gargalo, capacidadeResidual);
        }

        // Atualiza o fluxo para as arestas do caminho
        for (int v = qtdVertices - 1; v != 0; v = pai[v]) {
            int u = pai[v];
            fluxo[u][v] += gargalo; // Aumenta o fluxo na aresta direta
            fluxo[v][u] -= gargalo; // Ajusta o fluxo na aresta reversa
        }

        fluxo_maximo += gargalo; // Adiciona o fluxo do caminho aumentador ao fluxo total
    }

    return fluxo_maximo; // Retorna o fluxo máximo encontrado
}


void marcaVisitadosFecho(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int vertice, vector<bool>& visitados){
    visitados[vertice] = true;

    for (auto& elemento : listaAdj[vertice])
    {
        int vizinho = elemento.second.first;
        if (!visitados[vizinho])
        {
            marcaVisitadosFecho(listaAdj, vizinho, visitados);
        }
    }
}

vector <int> fechoTrans(vector<vector<pair<int, pair<int, int>>>>& listaAdj, int qtdVertices){
    vector<bool> visitadosBool(qtdVertices, false);
    vector<int> fecho;
    for (auto& relacao : listaAdj[0])
    {
        int verticeVizinho = relacao.second.first;
        marcaVisitadosFecho(listaAdj, verticeVizinho, visitadosBool);
    }

    for (size_t i = 0; i < visitadosBool.size(); i++)
    {
        if(visitadosBool[i] == true) fecho.push_back(i);
    }

    sort(fecho.begin(), fecho.end());
    return fecho;
}


int main(){
    queue<int> questoes;
    string linha;

    getline(cin, linha);
    stringstream ss(linha);
    int numero;
    while (ss >> numero)
    {
        questoes.push(numero);
    }


    int qtdVertices, qtdArestas;
    string tipoGrafo;
    cin >> qtdVertices >> qtdArestas >> tipoGrafo;

    // Inicialize lista_adj com o número de vértices
    vector<vector<pair<int, pair<int, int>>>> lista_adj(qtdVertices);



    for(int i = 0; i < qtdArestas; i++){
        int id, origem, destino, peso;
        cin >> id >> origem >> destino >> peso;
        lista_adj[origem].push_back(make_pair(id, make_pair(destino, peso)));
        if(tipoGrafo == "nao_direcionado"){
            lista_adj[destino].push_back(make_pair(id, make_pair(origem, peso)));
        }
    }

    bool ehConexo, ehBipartido, teste, cicloVar;
    int qtdComp, qtdPontes, qtdCompConex;
    vector<int> qtdArticulado, arestasDfs, arestasBfs, fecho;
    while(!questoes.empty()){
        int x = questoes.front();
        questoes.pop();
        switch(x){
            case 0:
                //1 -Verificar se um grafo é conexo (para o caso de grafos orientados, verificar conectividade fraca.)   
                ehConexo = conexoNaoDir(lista_adj, qtdVertices);
                cout << ehConexo << endl;
                break;

            case 1:
                //2 -Verificar se um grafo não-orientado é bipartido. 
                ehBipartido = bipartido(lista_adj, qtdVertices);
                cout << ehBipartido << endl;
                break;

            case 2:
                //3 -Verificar se um grafo qualquer é Euleriano.   
                teste = euleriano(lista_adj, qtdVertices);
                cout << teste << endl;
                break;

            case 3:
                //4 -Verificar se um grafo possui ciclo.
                cicloVar = ciclo(lista_adj, qtdVertices);
                cout<<cicloVar<<endl;
                break;

            case 4:
                //5 -Calcular a quantidade de componentes conexas em um grafo não-orientado.
                if (tipoGrafo != "direcionado")
                {
                    qtdComp = compConexo(lista_adj, qtdVertices); 
                    cout << qtdComp << endl;
                }else cout << -1 << endl;   
                break;

            case 5:
                //6 -Calcular a quantidade de componentes fortemente conexas em um grafo orientado.
                if (tipoGrafo == "direcionado")
                {
                    qtdCompConex = compConexo(lista_adj, qtdVertices);
                    cout << qtdCompConex << endl;
                }else cout << -1 << endl;                            
                break;

            case 6:
                //7 -Imprimir os vértices de articulação de um grafo não-orientado (priorizar a ordem lexicográfica dos vértices).    
                qtdArticulado = articulados(lista_adj, qtdVertices);
                if (tipoGrafo != "direcionado")
                {
                    if (!qtdArticulado.empty())
                    {
                    for (int vertices : qtdArticulado)
                    {
                        cout << vertices << " ";
                    }
                    cout << endl;
                    }else cout << 0 << endl;
                }else cout << -1 << endl;   

                break;

            case 7:            
                //8 -Calcular quantas arestas ponte possui um grafo não-orientado.  
                if (tipoGrafo != "direcionado")
                {
                    qtdPontes = pontes(lista_adj, qtdVertices);  
                    cout << qtdPontes << endl;
                }else cout << -1 << endl;   

                break;

            case 8:
                //9 -Imprimir a árvore em profundidade (priorizando a ordem lexicográfica dos vértices; 0 é a origem). Você deve imprimir o identificador das arestas. Caso o grafo seja desconexo, considere apenas a árvore com a raíz 0.   
                arestasDfs = dfs(lista_adj, qtdVertices);
                for(int id : arestasDfs) {
                    cout << id << " ";
                }
                cout<<endl;
                break;

            case 9:
                //10 -Árvore de largura (priorizando a ordem lexicográfica dos vértices; 0 é a origem). Você deve imprimir o identificador das arestas. Caso o grafo seja desconexo, considere apenas a árvore com a raíz 0.    
                arestasBfs = bfs(lista_adj, qtdVertices);
                for(int id : arestasBfs) {
                    cout << id << " ";
                }
                cout<<endl;
                break;

            case 10:
                //11 -Calcular o valor final de uma árvore geradora mínima (para grafos não-orientados).
                if(!conexoNaoDir(lista_adj, qtdVertices) or tipoGrafo == "direcionado")cout << "-1" << endl;
                else{
                    int pesoTotal = 0;
                    prim(lista_adj, qtdVertices, pesoTotal);
                    cout << pesoTotal << endl; 
                }    
                break;

            case 11:
                //12 -Imprimir a ordem os vértices em uma ordenação topológica. Esta função não fica disponível em grafos não direcionado. Deve-se priorizar a ordem lexicográfica dos vértices.   
                if (tipoGrafo == "nao_direcionado")
                {
                    cout << -1 << endl;
                }else{
                    vector<int> top = ordenacao_topologica(lista_adj,qtdVertices);
                    for(int id : top) {
                        cout << id << " ";
                    }
                    cout<<endl;
                }
                break;

            case 12:
                //13 -Valor do caminho mínimo entre dois vértices (para grafos não-orientados com pelo menos um peso diferente nas arestas).  0 é a origem; n-1 é o destino.   
                if(!conexoNaoDir(lista_adj, qtdVertices) or tipoGrafo == "direcionado")cout << "-1" << endl;
                else{  
                    int caminhoMin = djikstra(lista_adj, qtdVertices);
                    cout << caminhoMin << endl; 
                }   
                break;

            case 13: 
                //14 -Valor do fluxo máximo para grafos direcionados. 0 é a origem; n-1 é o destino.   
                if (tipoGrafo == "direcionado")
                {
                    cout << fluxoMaximo(lista_adj,qtdVertices) << endl;
                }else cout << -1 << endl; 
                break;

            case 14:
                //15 -Fecho transitivo para grafos direcionados.  Deve-se priorizar a ordem lexicográfica dos vértices; 0 é o vértice escolhido.
                if (tipoGrafo == "direcionado")
                {
                    fecho = fechoTrans(lista_adj, qtdVertices);
                    for (size_t i = 0; i < fecho.size(); i++)
                    {
                        cout << fecho[i] << " ";
                    }
                    cout << endl;
                }else cout << -1 << endl;   
                break;
        }
    }
    return 0;
}