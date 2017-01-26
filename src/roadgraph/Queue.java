package roadgraph;

import java.util.LinkedList;
import java.util.List;

public class Queue<E> {

	private List<E> queue;
	private Integer size;

	public Queue() {
		queue = new LinkedList<>();
		size = 0;
	}

	public boolean enQueue(E member) {
		size++;
		return queue.add(member);
	}

	public E deQueue() {
		if (isEmpty())
			return null;
		size--;
		return queue.remove(0);
	}

	public boolean isEmpty() {
		return queue.isEmpty();
	}

}
