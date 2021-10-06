#include<Siv3D.hpp>
#include<Eigen/Core>

using ScoreTable = HashTable<Point, HashTable<Point, HashTable<int, HashTable<int, double>>>>;	//table[p1][p2][e1][e2]
class Edge {
public:
	Eigen::MatrixX3d value;

	struct Score {
		const Edge* target;
		double score;
	};
	Array<Score> scores;
};

Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ImageToMatrix(const Image& image) {
	Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ret(image.width(), image.height());
	for (const auto [x, y] : step(image.size())) {
		const auto [r, g, b, a] = ColorF(image[y][x]);
		ret(y, x) << r, g, b;
	}
	return ret;
}
//template<class Derived>
std::array<Edge, 4> PickEdges(const Eigen::MatrixXd& panel) {
	std::array<Edge, 4> ret;
	std::array<Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, 1>, 4> edge = {
		panel.topRows(1).row(0).transpose().eval(),
		panel.rightCols(1).col(0).eval(),
		panel.bottomRows(1).row(0).transpose().reverse().eval(),
		panel.leftCols(1).col(0).reverse().eval()
	};

	for (int i : step(4)) {
		Eigen::MatrixX3d mat(panel.rows(), 3);
		for (int j : step(panel.rows())) {
			mat.row(j) = edge[i][j];
		}
		ret[i] = Edge{ mat };
	}
	return ret;
}

ScoreTable MakeScoreTable(const Grid<std::array<Edge, 4>>& edges) {
	ScoreTable table;
	const auto score_calc = [&](const Point& p1, int e1, const Point& p2, int e2) {
		return (edges[p1][e1].value - edges[p2][e2].value.colwise().reverse()).rowwise().squaredNorm().sum();
	};

	for (auto i : step(edges.size())) {
		for (auto j : step(edges.size())) {
			for (auto k : step(4)) {
				for (int l : step(4)) {
					table[i][j][k][l] = score_calc(i, k, j, l);
				}
			}
		}
	}

	return table;
}

class PPMInfo {
public:
	PPMInfo(FilePathView filepath) :
		m_reader(filepath) {
		if (not m_reader)return;
		m_reader.readLine();
		auto t = m_reader.readLine().value().split(U' ');
		divit.x = ParseInt<int>(t[1]);
		divit.y = ParseInt<int>(t[2]);
		t = m_reader.readLine().value().split(U' ');
		select_num = ParseInt<int>(t[1]);
		t = m_reader.readLine().value().split(U' ');
		cost.select = ParseInt<int>(t[1]);
		cost.swap = ParseInt<int>(t[2]);
	}
private:
	TextReader m_reader;
public:
	Size divit;
	int select_num;
	struct Cost {
		int select;
		int swap;
	}cost;
};

class Tile {
public:
	Tile(const Image& image, const Rect& region) :
		image(image),
		texture(image),
		matrix(ImageToMatrix(image)),
		edges(PickEdges(matrix)),
		region(region) {}
	void draw()const {
		texture.resized(region.size).rotated(rotate * Math::HalfPi).draw(region.pos);
	}
public:
	Image image;
	Texture texture;
	Eigen::MatrixXd matrix;
	std::array<Edge, 4> edges;
	Rect region;

	int rotate = 0;
};

void Main() {
	Window::SetStyle(WindowStyle::Sizable);

	constexpr FilePathView path = U"testcase/problem2.ppm";
	const PPMDecoder decoder;
	const PPMInfo info(path);

	const Image image = decoder.decode(path);
	const auto images = Grid<Image>::IndexedGenerate(info.divit, [&](const Point& p) {return image.clipped(Rect(image.size() / info.divit * p, image.size() / info.divit)); });
	const auto tiles = Grid<Tile>::IndexedGenerate(images.size(), [&](const Point& p) {return Tile(images[p], Rect(image.size() / info.divit * p, image.size() / info.divit)); });
	const auto score_table = MakeScoreTable(tiles >> [](const Tile& tile) {return tile.edges; });
	for (const auto& [p1, v1] : score_table) {
		for (const auto& [p2, v2] : v1) {
			for (const auto& [e1, v3] : v2) {
				for (const auto& [e2, score] : v3) {
					const_cast<Grid<Tile>&>(tiles)[p1].edges[e1].scores.push_back(Edge::Score{ &tiles[p2].edges[e2],score_table.at(p1).at(p2).at(e1).at(e2) });
				}
			}
		}
	}

	Texture texture(image);
	RenderTexture rt(4000, 2000, Scene::DefaultBackgroundColor);

	while (System::Update()) {
		rt.clear(Scene::DefaultBackgroundColor);

		texture.resized(400, 400).draw();
	}
}
