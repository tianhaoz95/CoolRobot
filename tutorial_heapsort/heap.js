
// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element);
    heapify(heap, heap.length - 1);
}

function heapify(heap, last_element) {
	var parent = 0;
	if (last_element == 0) {
		return;
	}
	if (last_element % 2) {
		parent = (last_element - 1) / 2;
	}
	else {
		parent = (last_element - 2) / 2;
	}
	if (heap[last_element] < heap[parent]) {
		swap(heap, last_element, parent);
		heapify(heap, parent);
	}
	return;
}

function swap(heap, first, second) {
	var buffer = heap[first];
	heap[first] = heap[second];
	heap[second] = buffer;
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var result = heap[0];
    swap(heap, 0, heap.length - 1);
    heap.pop();
    restore(heap, 0);
    return result;
}

function restore(heap, first_element) {
	var left_child = first_element * 2 + 1;
	var right_child = first_element * 2 + 2;
	var range = heap.length - 1;
	if (left_child > range || right_child > range) {
		return;
	}
	if (heap[first_element] > heap[left_child] && heap[left_child] < heap[right_child]) {
		swap(heap, first_element, left_child);
		restore(heap, left_child);
	}
	if (heap[first_element] > heap[right_child] && heap[right_child] < heap[left_child]) {
		swap(heap, first_element, right_child);
		restore(heap, right_child);
	}
}
// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object
