import java.util.*;

// Classe que representa uma cidade
class City {
    String name; // Nome da cidade
    int distanceToBoaVista; // Distância até boaVista
    List<Neighbor> neighbors; // Lista de vizinhos

    // Construtor da classe City
    public City(String name, int distanceToBoaVista) {
        this.name = name;
        this.distanceToBoaVista = distanceToBoaVista;
        this.neighbors = new ArrayList<>();
    }

    // Método para adicionar um vizinho com sua respectiva distância
    public void addNeighbor(City city, int distance) {
        this.neighbors.add(new Neighbor(city, distance));
    }

    // Sobrescrevendo o método toString para imprimir o nome da cidade
    @Override
    public String toString() {
        return name;
    }
}

// Classe que representa um vizinho de uma cidade
class Neighbor {
    City city; // Cidade vizinha
    int distance; // Distância até a cidade vizinha

    // Construtor da classe Neighbor
    public Neighbor(City city, int distance) {
        this.city = city;
        this.distance = distance;
    }
}

// Classe que representa um nó no algoritmo de busca
class Node implements Comparable<Node> {
    City city; // Cidade associada ao nó
    int cost; // Custo acumulado para alcançar esta cidade
    int heuristic; // Valor heurístico para estimar o custo restante até o destino
    int priority; // Prioridade usada na fila de prioridade

    // Construtor da classe Node
    public Node(City city, int cost, int heuristic) {
        this.city = city;
        this.cost = cost;
        this.heuristic = heuristic;
        this.priority = this.cost + this.heuristic;
    }

    // Construtor adicional para aceitar a prioridade como argumento
    public Node(City city, int cost, int heuristic, int priority) {
        this.city = city;
        this.cost = cost;
        this.heuristic = heuristic;
        this.priority = priority;
    }

    // Método de comparação para ordenação na fila de prioridade
    @Override
    public int compareTo(Node node) {
        return Integer.compare(this.priority, node.priority);
    }
}

// Classe principal que encontra o caminho mais curto até boaVista
public class RouteFinder {

    public static void main(String[] args) {
        // Criação das cidades e definição de suas distâncias até boaVista
        City uiramuta = new City("uiramuta", 194);
        City altoAlegre = new City("altoAlegre", 63);
        City boaVista = new City("boaVista", 0);
        City pacaraima = new City("pacaraima", 169);
        City normandia = new City("normandia", 160);
        City amajari = new City("amajari", 103);
        City bomfim = new City("bomfim", 88);
        City canta = new City("canta", 46);
        City caracarai = new City("caracarai", 139);
        City iracema = new City("iracema", 100);
        City caroebe = new City("caroebe", 266);
        City mucajai = new City("mucajai", 63);
        City saoJoaoDaBaliza = new City("saoJoaoDaBaliza", 232);
        City saoLuiz = new City("saoLuiz", 248);
        City rorainopolis = new City("rorainopolis", 233);

        // Adição dos vizinhos e suas respectivas distâncias
        uiramuta.addNeighbor(pacaraima, 187);
        pacaraima.addNeighbor(uiramuta, 187);

        altoAlegre.addNeighbor(boaVista, 84);
        boaVista.addNeighbor(altoAlegre, 84);

        bomfim.addNeighbor(normandia, 131);
        bomfim.addNeighbor(boaVista, 112);
        normandia.addNeighbor(bomfim, 131);
        boaVista.addNeighbor(bomfim, 112);

        canta.addNeighbor(boaVista, 36);
        boaVista.addNeighbor(canta, 36);

        caracarai.addNeighbor(iracema, 44);
        caracarai.addNeighbor(saoLuiz, 171);
        caracarai.addNeighbor(rorainopolis, 157);
        iracema.addNeighbor(caracarai, 44);
        saoLuiz.addNeighbor(caracarai, 171);
        rorainopolis.addNeighbor(caracarai, 157);

        mucajai.addNeighbor(boaVista, 58);
        boaVista.addNeighbor(mucajai, 58);

        saoJoaoDaBaliza.addNeighbor(saoLuiz, 17);
        saoJoaoDaBaliza.addNeighbor(caroebe, 25);
        saoLuiz.addNeighbor(saoJoaoDaBaliza, 17);
        caroebe.addNeighbor(saoJoaoDaBaliza, 25);

        amajari.addNeighbor(pacaraima, 164);
        amajari.addNeighbor(boaVista, 156);
        pacaraima.addNeighbor(amajari, 164);
        boaVista.addNeighbor(amajari, 156);

        iracema.addNeighbor(mucajai, 39);
        mucajai.addNeighbor(iracema, 39);

        saoLuiz.addNeighbor(boaVista, 245);
        boaVista.addNeighbor(saoLuiz, 245);

        // Seleção da cidade de partida
        City startingCity = saoJoaoDaBaliza;

        // Encontrar o caminho mais curto para boaVista
        List<City> shortestPath = findShortestPath(startingCity, boaVista);
        System.out.println("Caminho mais curto: " + shortestPath);
    }

    // Método para encontrar o caminho mais curto usando o algoritmo A*
    public static List<City> findShortestPath(City start, City destination) {
        // Fila de prioridade para nós a serem explorados
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        // Mapeamento de cidades anteriores para reconstruir o caminho
        Map<City, City> cameFrom = new HashMap<>();
        // Mapeamento de custos acumulados
        Map<City, Integer> costSoFar = new HashMap<>();

        // Adiciona o nó inicial à fila de prioridade
        openSet.add(new Node(start, 0, calculateHeuristic(start, destination)));
        // Inicializa o custo acumulado do nó inicial como 0
        costSoFar.put(start, 0);

        // Loop enquanto houver nós na fila de prioridade
        while (!openSet.isEmpty()) {
            // Remove o nó com o menor custo acumulado da fila de prioridade
            Node current = openSet.poll();

            // Verifica se o nó removido é a cidade de destino
            if (current.city.equals(destination)) {
                // Se sim, retorna o caminho reconstruído
                return reconstructPath(cameFrom, destination);
            }

            // Para cada vizinho da cidade atual
            for (Neighbor neighbor : current.city.neighbors) {
                // Calcula o custo tentativo para alcançar este vizinho
                int newCost = costSoFar.get(current.city) + neighbor.distance;

                // Se o custo tentativo for menor do que o custo atualmente registrado para este
                // vizinho
                if (!costSoFar.containsKey(neighbor.city) || newCost < costSoFar.get(neighbor.city)) {
                    // Atualiza o custo registrado para este vizinho
                    costSoFar.put(neighbor.city, newCost);

                    // Calcula a heurística para este vizinho
                    int heuristic = calculateHeuristic(neighbor.city, destination);

                    // Calcula a prioridade do nó (custo acumulado + heurística)
                    int priority = newCost + heuristic;

                    // Adiciona o nó à fila de prioridade
                    openSet.add(new Node(neighbor.city, newCost, heuristic, priority));

                    // Registra a cidade anterior para este vizinho
                    cameFrom.put(neighbor.city, current.city);
                }
            }
        }

        // Se não foi possível alcançar a cidade de destino a partir da cidade inicial
        return null;
    }

    // Método para reconstruir o caminho mais curto
    public static List<City> reconstructPath(Map<City, City> cameFrom, City current) {
        List<City> path = new ArrayList<>();
        // Adiciona a cidade atual ao caminho
        path.add(current);
        // Enquanto houver cidades anteriores registradas para a cidade atual
        while (cameFrom.containsKey(current)) {
            // Obtém a cidade anterior da cidade atual
            current = cameFrom.get(current);
            // Adiciona a cidade anterior ao caminho
            path.add(current);
        }
        // Inverte a ordem do caminho para que seja da origem ao destino
        Collections.reverse(path);
        return path;
    }

    // Método para calcular a heurística entre duas cidades
    public static int calculateHeuristic(City current, City destination) {
        return Math.abs(current.distanceToBoaVista - destination.distanceToBoaVista);
    }
}
