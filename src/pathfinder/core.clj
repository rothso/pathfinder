(ns pathfinder.core
  (:require [clojure.data.priority-map :refer [priority-map, priority-map-by]])
  (:require [clojure.edn :as edn])
  (:require [clojure.math.combinatorics :as combo])
  (:require [clojure.math.numeric-tower :as math])
  (:require [clojure.set :as set])
  (:require [quil.core :as q])
  (:gen-class))

(defn rotate [n a]
  (let [l (count a)
        off (mod (+ (mod n l) l) l)]
    (concat (drop off a) (take off a))))

(defn update-map [f m]
  (reduce-kv (fn [m k v]
               (assoc m k (f k v))) {} m))

(defn distance [vert-a vert-b]
  (math/sqrt (reduce + (map #(math/expt % 2) (map - vert-a vert-b)))))

(defn create-edge [a b]
  {:from a :to b})

(defn flip [edge]
  (set/rename-keys edge {:from :to, :to :from}))

(defn neighboring-edges [verts]
  (map create-edge verts (rotate 1 verts)))

(defn ccw [[ax ay] [bx by] [cx cy]]
  (> (* (- cy ay) (- bx ax))
     (* (- by ay) (- cx ax))))

(defn intersecting? [a b c d]
  (and (not= (ccw a c d) (ccw b c d))
       (not= (ccw a b c) (ccw a b d))
       (and (distinct? a b c d))))

(defn shape-verts [shapes]
  (reduce-kv (fn [m k v] (conj m (map (fn [x] [k x]) v))) [] shapes))

(defn obstacle-edges [shapes]
  (set (flatten (map neighboring-edges (shape-verts shapes)))))

(defn search-space [shapes goal]
  (conj (apply concat (shape-verts shapes)) goal))

(defn visible? [obstacle-edges {[_ a] :from [_ b] :to}]
  (not-any? (fn [{[_ c] :from [_ d] :to}] (intersecting? a b c d)) obstacle-edges))

(defn valid-edge? [obstacle-edges {[from] :from [to] :to :as edge}]
  (or (not= from to)                                        ;; going to different shape
      (contains? obstacle-edges edge)                       ;; adjacent on the same shape
      (contains? obstacle-edges (flip edge))))              ;; adjacent on the same shape

(defn get-visible-vertices [shapes goal vertex]
  (let [obstacle-edges (obstacle-edges shapes)]
    (->> (map #(create-edge vertex %) (search-space shapes goal))
         (filter (partial valid-edge? obstacle-edges))      ;; not going through the shape
         (filter (partial visible? obstacle-edges))         ;; not going through other shapes
         (map :to))))

(defn h [n goal]
  (distance (second n) (second goal)))

(defn g [current-travelled from to]
  (+ current-travelled (distance from to)))

(defn A* [shapes start goal]
  (loop [open-list (priority-map start (h start goal))
         closed-list #{}
         came-from {}
         g-score (assoc (zipmap (search-space shapes goal) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [current (key (peek open-list))
            neighbors (->> current                          ;; get all unvisited neighbors
                           (get-visible-vertices shapes goal)
                           (filter (comp not (partial contains? closed-list))))
            g-score' (->> neighbors                         ;; get improved g-scores of neighbors
                          (map #(g (second (get g-score current)) (second current) %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            f-score' (update-map #(+ %2 (h %1 goal)) g-score')]
        (if (= goal current)
          (loop [current goal path []]
            (if (= current nil)
              (reverse path)
              (recur (get came-from current) (conj path current))))
          (recur
            (into (pop open-list) f-score')                 ;; add improved neighbors to open list
            (conj closed-list current)                      ;; add current vertex to the closed list
            (merge came-from (zipmap (keys g-score') (repeat current))) ;; update path to neighbor
            (merge-with min g-score g-score')))))))         ;; update the g-scores with best scores

(defn u [c g-score h-score]
  (if (= h-score 0) ##Inf (/ (- c g-score) h-score)))

(defn potential-search [shapes start goal cost]
  (loop [open-list (priority-map-by > start (u cost 0 (h start goal)))
         closed-list #{}
         came-from {}
         g-score (assoc (zipmap (search-space shapes goal) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [current (key (peek open-list))
            neighbors (->> current                          ;; get all unvisited neighbors
                           (get-visible-vertices shapes goal)
                           (filter (comp not (partial contains? closed-list))))
            g-score' (->> neighbors                         ;; get improved g-scores of neighbors
                          (map #(g (second (get g-score current)) (second current) %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            u-score' (update-map #(u cost %2 (h %1 goal)) g-score')]
        (if (>= (val (peek open-list)) 0)                   ;; u(n) < 0, no path found, return nil
          (if (= goal current)
            (loop [current goal path []]
              (if (= current nil)
                (reverse path)
                (recur (get came-from current) (conj path current))))
            (recur
              (into (pop open-list) u-score')               ;; add improved neighbors to open list
              (conj closed-list current)                    ;; add current vertex to the closed list
              (merge came-from (zipmap (keys g-score') (repeat current))) ;; update path to neighbor
              (merge-with min g-score g-score'))))))))      ;; update the g-scores with best scores

(defn cost [path]
  ;(reduce + (map #(apply distance %) (partition 2 1 (map second path)))))
  (reduce + (map #(apply distance %) (partition 2 1 path))))

(defn get-manhattan-neighbors [[x y]]
  (list [(+ x 1) y]
        [(- x 1) y]
        [x (+ y 1)]
        [x (- y 1)]))

(defn ARA*-improve [n obs start goal G w]
  (loop [open-list (priority-map start (* w (distance start goal)))
         came-from {}
         g-score (assoc (zipmap (combo/cartesian-product (range n) (range n)) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [current (key (peek open-list))
            neighbors (->> current                          ;; get all successors
                           (get-manhattan-neighbors)
                           (filter (fn [[x y]] (and (<= 0 x (- n 1)) (<= 0 y (- n 1)))))
                           (filter (comp not (partial contains? obs)))
                           (filter (comp not (partial contains? open-list))))
            g-score' (->> neighbors                         ;; get improved g-scores of neighbors
                          (map #(g (get g-score current) current %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            f-score' (->> g-score'
                          (filter #(< (+ (val %) (distance (key %) goal)) G)) ;; only if g(n') + h(n') < G
                          (into {})
                          (update-map #(+ %2 (* w (distance %1 goal)))))]
        (if (<= (val (peek open-list)) G)                   ;; fw(n) >= G, no path found, return nil
          (if (= goal current)
            (loop [current goal path []]
              (if (= current nil)
                [open-list (reverse path)]                  ;; pass the state of the open-list back
                (recur (get came-from current) (conj path current))))
            (recur
              (into (pop open-list) f-score')               ;; add improved neighbors to open list
              (merge came-from (zipmap (keys g-score') (repeat current))) ;; update path to neighbor
              (merge-with min g-score g-score'))))))))      ;; update the g-scores with best scores

(defn ARA* [n obs start goal w0 dw]
  (loop [open-list (priority-map start (* w0 (distance start goal)))
         incumbent nil
         G ##Inf
         w w0]
    (if (not (empty? open-list))
      (let [new-solution (ARA*-improve n obs start goal G w)]
        (if (some? new-solution)
          (let [[open-list' path] new-solution]
            (recur open-list' path (cost path) (- w dw)))
          incumbent))
      incumbent)))

(defn scale [x y]
  [(* x 20) (- (q/height) (* y 20))])

(defn draw [shapes start goal path]
  (let [[start-x start-y] (second start)
        [goal-x goal-y] (second goal)]
    (q/with-translation
      [50 5]
      (q/background 255)
      (q/stroke 0 0 0)
      (q/stroke-weight 2)
      (doseq [shape (vals shapes)]                          ;; draw each obstacle
        (q/begin-shape)
        (doseq [[x y] shape]
          (apply q/vertex (scale x y)))
        (q/end-shape :close))
      (q/stroke 255 0 0)
      (q/stroke-weight 10)
      (apply q/point (scale start-x start-y))               ;; draw start point
      (apply q/point (scale goal-x goal-y))                 ;; draw goal point
      (q/stroke-weight 2)
      (q/no-fill)
      (q/begin-shape)                                       ;; draw search path
      (doseq [[_ [x y]] path]
        (apply q/vertex (scale x y)))
      (q/end-shape))))

(defn prompt-read [prompt]
  (print (format "%s: " prompt))
  (flush)
  (read-line))

(def environments
  [{:shapes {:a [[2 6] [17 6] [17 1] [2 1]]
             :b [[0 14] [6 19] [9 15] [7 8] [1 9]]
             :c [[10 8] [12 15] [14 8]]
             :d [[14 19] [18 20] [21 17] [14 13]]
             :e [[18 10] [23 6] [19 3]]
             :f [[22 19] [28 19] [28 9] [22 9]]
             :g [[25 6] [28 8] [31 6] [31 2] [28 1] [25 2]]
             :h [[29 17] [31 19] [34 16] [32 8]]}
    :start  [:start [1 3]]
    :goal   [:goal [34 19]]},
   {:shapes {:a [[12 4] [18 4] [15 6]]
             :b [[8 4] [15 8] [22 4] [22 10] [8 10]]
             :c [[3 7] [6 7] [3 10] [6 10]]
             :d [[3 14] [12 14] [12 16] [3 16]]
             :e [[15 14] [26 14] [30 18] [19 18]]}
    :start  [:start [15 1]]
    :goal   [:goal [15 18]]}])

(defn process [choice]
  (let [{shapes :shapes
         start  :start
         goal   :goal} (nth environments (- choice 1))
        w (edn/read-string (prompt-read "Enter the initial weight (w0)"))
        dw (edn/read-string (prompt-read "Enter the change in weight (dw)"))
        path (ARA* 100 shapes start goal w dw)]
    (if (nil? path)
      (println "No path found")
      (do
        ;; print the search path to the console
        (doall (map println path))
        ;; draw the search path using quil
        (q/sketch
          :title (str "Environment " choice)
          :size [800 400]
          :renderer :p2d
          :draw #(do (draw shapes start goal path)
                     ;(q/save (str "PTS_Rothanak_So_" choice ".png"))
                     ))))))

(defn -main []
  (loop []
    (let [prompt "Select an environment (1-2) or -1 to quit"
          choice (edn/read-string (prompt-read prompt))]
      (cond (<= 1 choice 2) (do (process choice) (recur))
            (= choice -1) (println "Goodbye!")
            :else (recur)))))

(defn draw-grid [path obs [start-x start-y] [goal-x goal-y] w]
  (q/background 255)                                        ; background color
  (q/stroke 255)                                            ; cell border color
  (q/fill 255 0 0)
  (doseq [[x y] path]
    (q/rect (* w x) (* w y) w w 2))
  (q/fill 0)
  (doseq [[x y] obs]
    (q/rect (* w x) (* w y) w w 2))
  (q/fill 0 255 0)
  (q/rect (* w start-x) (* w start-y) w w 2)
  (q/fill 0 0 255)
  (q/rect (* w goal-x) (* w goal-y) w w 2))

(defn -main []
  (let [resolution 1000
        n (edn/read-string (prompt-read "Enter the grid height/width (n)"))
        p (edn/read-string (prompt-read "Enter the obstacle density (p)"))
        w 0.8
        dw 0.2
        start [0 0]
        goal [(- n 1) (- n 1)]
        ;obs (set/difference (set (map (fn [_] [(rand-int n) (rand-int n)]) (range (* n n p)))) #{start goal})
        obs (set (map (fn [_] [(+ (rand-int (- n 1)) 1) (+ (rand-int (- n 1)) 1)]) (range (* n n p))))
        path (ARA* n obs start goal w dw)]
    (println path)
    (println (cost path))
    (q/sketch
      :title "HELP"
      :size [resolution resolution]
      :draw #(draw-grid path obs start goal (/ resolution n)))))