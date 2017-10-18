/* ESTE CÓDIGO UTILIZA C++11; Na compilação usar: g++ novocpo.cpp -o novocpo -pthread -std=c++0x */
/*
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cmath>
#include <cfloat>
#include <vector>
#include <string>
#include <fstream>

#include <thread>
#include <chrono>
#include <sys/time.h>
#include <condition_variable>
#include <mutex>

#define ENEMY 1
#define FRIEND 2
#define FreeCell 2.0
#define OccupiedCell 1.0
#define MaxPotencial 1.0
#define MinPotencial 0.0
#define PotencialInicial 0.995
#define Epsilon 1.0
#define itMax 1000
#define ErrorMin 1e-12

using namespace std;

//Variável responsável por controlar o ínicio da execução dos algoritmos das threads
int FLAG_THREAD = 0;				//0=false; 1=true

//Variáveis globais que armazenarão os tempos inciais e finais de execução do algoritmo
struct timeval inicial;
struct timeval final;

//Variável global responsável por finalizar a execução do algoritmo
int finished = 0;

//Variáveis globais responsáveis por armazenar o erro
long double erroRC = 0.0;
long double erroBC = 0.0;

//Variáveis globais responsáveis por controlar a execução da Thread Erro
bool controleRC = false;			//TRUE=permite leitura; FALSE=aguarda atualização da variável
bool controleBC = false;			//TRUE=permite leitura; FALSE=aguarda atualização da variável
bool f_controleRC(){
	return (!controleRC);
}
bool f_controleBC(){
	return (!controleBC);
}

//Variáveis globais responsáveis por controlar o número de iterações de cada célula preta e vermelha
int roundRC = 0;
int roundBC = 0;

//Variáveis que controlam a execução das Threads
mutex mtx_bc;
mutex mtx_rc;
mutex mtx_erro;

condition_variable cv_bc;
condition_variable cv_rc;
condition_variable cv_erro;

unique_lock<mutex> lck_rc(mtx_rc);
unique_lock<mutex> lck_bc(mtx_bc);
unique_lock<mutex> lck_erro(mtx_erro);

//Função que informará se a Thread Erro está apta à processar o Erro das Threads Pretas e Vermelhas
bool f_ready(){
	return ((controleRC == true) && (controleBC == true));
}

//Classe Posição
class posicao{
	private:
		double x,y;
	public:
		//Construtores
		posicao(double x, double y){
			//Verificando se os dados da posição são válidos
			if(x >= 0)
				this->x = x;
			else
				this->x = 0;

			if(y >= 0)
				this->y = y;
			else
				this->y = 0;
		}
		posicao(){
			this->x = 0;
			this->y = 0;
		}

		//Setters
		void setX(double novoX){
			x = novoX;
		}
		void setY(double novoY){
			y = novoY;
		}
		void setXY(double novoX, double novoY){
			x = novoX;
			y = novoY;
		}

		//Getters
		double getX(){
			return (this->x);
		}
		double getY(){
			return (this->y);
		}
		void getXY(double x, double y){
			x = this->x;
			y = this->y;
		}
};

//Classe Robô
class robot{
	private:
		string nome;
		int tipo;			//ENEMY or FRIEND
		posicao *pos;

	public:
		//Construtores
		robot(int t, int px, int py, double v){
			//Verificando se os dados do tipo do robô são válidos
			if(t == ENEMY || t == FRIEND)
				this->tipo = t;
			else
				this->tipo = FRIEND;	//Valor padrão

			this->pos = new posicao(px, py);
		}
		robot(){
			this->tipo = FRIEND;		//Valor padrão
			this->pos = new posicao();
		}

		//Destrutores
		~robot(){
			delete pos;
		}

		//Setters
		void setNome(string x){
			if(x.size() != 0){
				this->nome = x;
			}
		}
		void setPosicao(int px, int py){
			pos->setXY(px, py);
		}
		void setTipo(int t){
			if(t == ENEMY || t == FRIEND)
				this->tipo = t;
		}

		//Getters
		string getNome(){
			return (this->nome);
		}
		int getTipo(){
			return (this->tipo);
		}
		void getPosicao(int pX, int pY){
			pX = this->pos->getX();
			pY = this->pos->getY();
		}
};

//Classe Ambiente
class amb_OPFE{
	private:
		int altura, largura, tamanho;		//Dimensoes ambiente 
		int nMetas, nObstaculos;
		double v, sin_v, cos_v;
		double epsilon;

		vector <posicao> metas, obstaculos;
		vector <robot> robos;
		//Matrizes alocadas como vetores lineares (insercao dos elementos sequencialmente)
		vector <double> malha_potencial, malha_ocupacao;	

	public:
		//Construtor
		amb_OPFE(int width, int height, double v, double epsilon){
			int i;

			//Armazenando as informacoes do novo ambiente 
			this->altura = height;
			this->largura = width;
			this->tamanho = height*width;
			this->nMetas = 0;
			this->nObstaculos = 0;
			this->epsilon = epsilon;
			this->v = v;
			this->sin_v = sin(v);
			this->cos_v = cos(v);

			//Alocando e preenchendo a memoria das malhas potenciais e de ocupacao
			for(i = 0; i < this->tamanho; i++){
				this->malha_potencial.push_back(PotencialInicial);
				this->malha_ocupacao.push_back(FreeCell);
			}
		}

		//Destrutor
		~amb_OPFE(){
			//Avalia-se se os vetores estão vazios ou não para realizar a liberação da memória alocada
			if(!this>metas.empty())
				this->metas.clear();
			if(!this>obstaculos.empty())
				this->obstaculos.clear();
			if(!this>robos.empty())
				this->robos.clear();
			if(!this>malha_potencial.empty())
				this->malha_potencial.clear();
			if(!this>malha_ocupacao.empty())
				this->malha_ocupacao.clear();
		}

		//Setters
		void setMalhaOcupacao(int i, int j, int valor){
			if((((j*this->largura) + i) < this->tamanho) && ((valor == FreeCell) || (valor == OccupiedCell)))
				this->malha_ocupacao[(j*this->largura) + i] = valor;
			else
				cout << "\t * ERROR! It's not possible to set Malha Ocupacao!\n\n";
		}
		void setMalhaPotencial(int i, int j, double valor){
			if(((j*this->largura) + i) < this->tamanho)
				this->malha_potencial[(j*this->largura) + i] = valor;
			else
				cout << "\t * ERROR! It's not possible to set Malha Potencial!\n\n";
		}
		void setEpsilon(double x){
			this->epsilon = x;
		}
		void addMeta(posicao x){
			int i,j;
			i = x.getX();
			j = x.getY();
			if((((j*this->largura) + i) < this->tamanho) && (this->malha_ocupacao[(j*this->largura) + i] == FreeCell)){
				this->malha_ocupacao[(j*this->largura) + i] = OccupiedCell;
				this->malha_potencial[(j*this->largura) + i] = MinPotencial;
				this->nMetas++;
				this->metas.push_back(x);
			}
			else
				cout << "\t * ERROR! It's not possible to add a new Goal!\n\n";
		}
		void addObstaculo(posicao x){
			int i, j;
			i = x.getX();
			j = x.getY();
			if((((j*this->largura) + i) < this->tamanho) && (malha_ocupacao[(j*this->largura) + i] == FreeCell)){
				this->malha_ocupacao[(j*this->largura) + i] = OccupiedCell;
				this->malha_potencial[(j*this->largura) + i] = MaxPotencial;
				this->nObstaculos++;
				this->obstaculos.push_back(x);
			}
			else
				cout << "\t * ERROR! It's not possible to add a new Obstacle!\n\n";
		}

		//Getters
		double getMalhaOcupacao(int i, int j){
			return (this->malha_ocupacao[(j*this->largura) + i]);
		}
		double getMalhaPotencial(int i, int j){
			return (this->malha_potencial[(j*this->largura) + i]);
		}
		int getAltura(){
			return (this->altura);
		}
		int getLargura(){
			return (this->largura);
		}
		int getTamanho(){
			return (this->tamanho);
		}
		double getEpsilon(){
			return (this->epsilon);
		}
		double getCosV(){
			return (this->cos_v);
		}
		double getSinV(){
			return (this->sin_v);
		}
		int getNumberOwnRobots(){
			int number = 0;

			for(int i = 0; i < (signed)robos.size(); i++){
				if(robos.at(i).getTipo() == FRIEND)
					number++;
			}

			return (number);
		}
		int getNumberEnemyRobots(){
			int number = 0;

			for(int i = 0; i < (signed)robos.size(); i++){
				if(robos.at(i).getTipo() == ENEMY)
					number++;
			}

			return (number);
		}

		//Printers
		void printMalhas(int x){		//1=somente Malha Ocupacao; 2=somente Malha Potencial; 3=ambas as malhas
			int i, j;
			if(x == 1 || x == 3){
				if(this->malha_ocupacao.size() != 0){
				cout << "\t# Malha Ocupacao\n";
				for(i = 0; i < this->altura; i++){
					for(j = 0; j < this->largura; j++){
						cout << "\t" << setiosflags (ios::fixed) << setprecision(0) << this->malha_ocupacao[i*this->largura + j];
					}
					cout << "\n";
				}
				cout << "\n\n";
				}
				else
					cout << "\t * ERROR! Malha Ocupacao not found! \n\n";
			}

			if(x == 2 || x == 3){
				if(this->malha_potencial.size() != 0){
					cout << "\t# Malha Potencial\n";
					for(i = 0; i < this->altura; i++){
						for(j = 0; j < this->largura; j++){
							cout << "\t" << setiosflags (ios::fixed) << setprecision(5) << this->malha_potencial[i*largura + j];
						}
						cout << "\n";
					}
					cout << "\n\n";
				}
				else
					cout << "\t * ERROR! Malha Potencial not found! \n\n";
			}

			if(x == 4){	//Impressão em arquivo
				ofstream outfile;
				outfile.open("CPOparalell.txt", ios::trunc);

				if(outfile.is_open()){
					if(this->malha_potencial.size() != 0){
						outfile << "\t# Malha Potencial\n";
						for(i = 0; i < this->altura; i++){
							for(j = 0; j < this->largura; j++){
								outfile << "\t" << setiosflags (ios::fixed) << setprecision(5) << this->malha_potencial[i*largura + j];
							}
							outfile << "\n";
						}
						outfile << "\n\n";
					}
					outfile.close();
				}
				else
					cout << "\t * ERROR! Falha ao abrir o arquivo! \n\n";

			}
		}
};

//Iterador Gauss Seidel usando diferencas centradas - Células vermelhas
void gauss_seidel_OPFE_rc(amb_OPFE *e){
	int x, i, j, largura_ambiente, altura_ambiente;
	double oldPotential, newPotential, left, right, top, bottom, lambda_vx, lambda_vy, epsAux;
	long double error;

   	//Armazenando as informações do ambiente
	largura_ambiente = e->getLargura();
	altura_ambiente = e->getAltura();

	//Calculando o valor de epsilon
	epsAux = e->getEpsilon() / 2;
	lambda_vx = (e->getEpsilon() * e->getCosV()) / 2;
	lambda_vy = (e->getEpsilon() * e->getSinV()) / 2;
	
	//Carregando os recursos das Threads
	this_thread::sleep_for(chrono::milliseconds(500));

	//Iniciando a contagem do tempo de execução do código
	gettimeofday(&inicial, NULL);

	//Executa o algoritmo enquanto a thread erro não mudar o status de finished para 1, indicando que terminamos o processo
	while(finished == 0){
		//Calculando o novo potencial
		error = 0.0;
		x = 1;
		for(i = 0; (i < altura_ambiente); i++){
			for(j = x; (j < largura_ambiente); j = j+2){     //Avaliando se estamos em uma célula vermelha
				if(e->getMalhaOcupacao(i,j) == FreeCell){
					oldPotential = e->getMalhaPotencial(i,j);

                    // Buscando a vizinhança da célula (i,j)
					//Analisando se estamos na fronteira da malha
					if(i == 0){
						bottom = MaxPotencial;
						top = e->getMalhaPotencial(i+1,j);
					}
					else {
						if((e->getAltura() - 1) == i){
							bottom = e->getMalhaPotencial(i-1,j);
							top = MaxPotencial;
						}
						else{
							bottom = e->getMalhaPotencial(i-1,j);
							top = e->getMalhaPotencial(i+1,j);
						}
					}

					//Analisando se estamos na fronteira da malha
					if(j == 0){
						left = MaxPotencial;
						right = e->getMalhaPotencial(i,j+1);
					}
					else {
						if((e->getLargura() - 1) == j){
							left = e->getMalhaPotencial(i,j-1);
							right = MaxPotencial;
						}
						else{
							left = e->getMalhaPotencial(i,j-1);
							right = e->getMalhaPotencial(i,j+1);
						}
					}

					//Calculando o novo potencial
					newPotential = ((1+lambda_vx)*right + (1-lambda_vx)*left + (1+lambda_vy)*top + (1-lambda_vy)*bottom);
					newPotential /= 4;
					
					//Alterando o valor do potencial
					e->setMalhaPotencial(i, j, newPotential);

					//Calculando o erro do método
					error += ((newPotential-oldPotential)*(newPotential-oldPotential));
				}
			}
			x = !x;
		}

		//Armazenando o erro das células vermelhas
		erroRC = error;

		//Contabilizando a rodada (iteração)
		roundRC++;

		//Sinal que houve mudança na variável erro
		controleRC = true;
		cv_erro.notify_one();

		//Esperando a resposta da Thread Erro para saber se encerramos o processo ou não
		cv_rc.wait(lck_rc, f_controleRC);
	}
}

//Iterador Gauss Seidel usando diferencas centradas - Células pretas
void gauss_seidel_OPFE_bc(amb_OPFE *e){
	int x, i, j, largura_ambiente, altura_ambiente;
	double oldPotential, newPotential, left, right, top, bottom, lambda_vx, lambda_vy, epsAux;
	long double error;

	//Armazenando as informações do ambiente
	largura_ambiente = e->getLargura();
	altura_ambiente = e->getAltura();

	//Calculando o valor de epsilon
	epsAux = e->getEpsilon() / 2;
	lambda_vx = (e->getEpsilon() * e->getCosV()) / 2;
	lambda_vy = (e->getEpsilon() * e->getSinV()) / 2;

	//Carregando os recursos das Threads
	this_thread::sleep_for(chrono::milliseconds(500));

	//Iniciando a contagem do tempo de execução do código
	gettimeofday(&inicial, NULL);

	//Executa o algoritmo enquanto a thread erro não mudar o status de finished para 1, indicando que terminamos o processo
	while(finished == 0){
		//Calculando o novo potencial
		error = 0.0;
		x = 0;
		for(i = 0; (i < altura_ambiente); i++){
			for(j = x; (j < largura_ambiente); j = j+2){    //Avaliando se estamos em uma célula preta
				if(e->getMalhaOcupacao(i,j) == FreeCell){
					oldPotential = e->getMalhaPotencial(i,j);

                    // Buscando a vizinhança da célula (i,j)
					//Analisando se estamos na fronteira da malha
					if(i == 0){
						bottom = MaxPotencial;
						top = e->getMalhaPotencial(i+1,j);
					}
					else {
						if((e->getAltura() - 1) == i){
							bottom = e->getMalhaPotencial(i-1,j);
							top = MaxPotencial;
						}
						else{
							bottom = e->getMalhaPotencial(i-1,j);
							top = e->getMalhaPotencial(i+1,j);
						}
					}

					//Analisando se estamos na fronteira da malha
					if(j == 0){
						left = MaxPotencial;
						right = e->getMalhaPotencial(i,j+1);
					}
					else {
						if((e->getLargura() - 1) == j){
							left = e->getMalhaPotencial(i,j-1);
							right = MaxPotencial;
						}
						else{
							left = e->getMalhaPotencial(i,j-1);
							right = e->getMalhaPotencial(i,j+1);
						}
					}

					//Calculando o novo potencial
					newPotential = ((1+lambda_vx)*right + (1-lambda_vx)*left + (1+lambda_vy)*top + (1-lambda_vy)*bottom);
					newPotential /= 4;
					
					//Alterando o valor do potencial
					e->setMalhaPotencial(i, j, newPotential);

					//Calculando o erro do método
					error += ((newPotential-oldPotential)*(newPotential-oldPotential));
				}
			}
			x = !x;
		}

		//Armazenando o erro das células pretas
		erroBC = error;

		//Contabilizando a rodada (iteração)
		roundBC++;

		//Sinal que houve mudança na variável erro
		controleBC = true;
		cv_erro.notify_one();
		
		//Esperando a resposta da Thread Erro para saber se encerramos o processo ou não
		cv_bc.wait(lck_bc, f_controleBC);
	}
}

//Função que monitora a matriz de erro
void monitora_erro(amb_OPFE *e){
	int k;
	long double error;

	//Inicializando variáveis
	error = 0.0;
	k = 0;

	//Thread espera pelo início da execução do GSOR
	this_thread::sleep_for(chrono::milliseconds(500));

	//Executa a verificação de erros até que error seja menor que o ErroMin ou até atingirmos o número máximo de iterações
	while(finished == 0){
		//Aguardando as Threads Pretas e Vermelhas informarem que houve modificação na variável erroRC e erroBC
		cv_erro.wait(lck_erro, f_ready);

		k = roundBC;

		//Obtendo o erro da iteração atual
		error = erroRC + erroBC;

		//Verificando se atingimos o erro mínimo ou o número máximo de iterações
		if((error < ErrorMin) || (k > itMax)){
			//Finalizando a contagem do tempo
			gettimeofday(&final, NULL);
			
			//Setando a variável para 1, finalizando o processo de execução do algoritmo
			finished = 1;
		}

		//Enviando o sinal que a Thread Erro terminou seu processamento
		controleBC = false;
		cv_bc.notify_one();
		controleRC = false;
		cv_rc.notify_one();

	}
	cout << k; 
}

//Solver Gauss-Seidel OPFE
void solver_OPFE(amb_OPFE *e){	
	//Declarando e incializando as threads
	thread blackThread(gauss_seidel_OPFE_bc, e);
	thread redThread(gauss_seidel_OPFE_rc, e);
	thread errorThread(monitora_erro, e);

	//Aguardando as threads acabarem
	blackThread.join();
	redThread.join();
	errorThread.join();
}
*/

/*
int main() {
	//Comando para não dar conflito entre as bibliotecas iostream e cstdio
	ios::sync_with_stdio(0);
	double duracao;
	posicao aux;

	//Declarando o ambiente
	amb_OPFE *robots = new amb_OPFE(10, 10, M_PI/2.0, 0.5);

	//Inserindo meta
	aux.setX(0);
	aux.setY(3);
	robots->addMeta(aux);
	//cout << "Posicao Meta: (" << aux.getX() << "," << aux.getY() << ")\n";

	//Inserindo obstaculo
	aux.setX(8);
	aux.setY(6);
	robots->addObstaculo(aux);
	//cout << "Posicao Obstaculo: (" << aux.getX() << "," << aux.getY() << ")\n";

	//Executando o algoritmo
	solver_OPFE(robots);

	//Imprimindo os resultados obtidos no arquivo
	robots->printMalhas(4);

	//Contabilizando o tempo de execução do algoritmo (resultado em segundos)
	duracao = ((1000000 * (final.tv_sec - inicial.tv_sec)) + (final.tv_usec - inicial.tv_usec))/1000;
	cout << " \t" << duracao << endl;

	//Liberando a memória alocada
	delete robots;

	return 0;
}
*/
