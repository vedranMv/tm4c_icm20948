#ifndef __LINKED_LIST__
#define __LINKED_LIST__

#include <memory>
#include <tuple>
#include <iostream>

using namespace std;


/**
 *  Node in a linked list
 *  Node has data part and two pointers, one to next and one to previous node
 */
class LLnode
{
    //typedef std::shared_ptr< LLnode<T> > p_LLnode;
    typedef LLnode* p_LLnode;
    
    public:
    LLnode(): _prev(nullptr), _next(nullptr) {};
    LLnode(float arg, uint8_t ind, p_LLnode prv=nullptr, p_LLnode nxt=nullptr): data(arg), index(ind), _prev(prv), _next(nxt) {};
    
    float       data;
    uint8_t     index;
    p_LLnode    _prev;
    p_LLnode    _next;
};

/**
 *  Sorted linked list of tuples (currently sorted in descending order)
 *  Inserting with addS -> linked list with elements sorted in desc order
 *  Inserting with addS(sorting key const.) -> linked list behaves as stack(LIFO)
 *  Inserting with addQ -> linked list behaves as queue
 */
class LinkedList
{
    //  Shorten the declaration of shared pointer
    //typedef std::shared_ptr< LLnode<T> > p_LLnode;
    typedef LLnode* p_LLnode;
    public:
        LinkedList(): head(nullptr), tail(nullptr), siz(0), refExists(false) {};
        ~LinkedList()
        {
            if (!refExists)
            {
                //  If no reference to this list exists delete is completly
                while (!LinkedList::empty())
                    popFront();
            }
            //  Else don't delete it because its members are referenced in
            //  another list
        }
        
        ///  True if arg1 is smaller or equal to arg2 - sorting in desc order
        bool sortLogic (const p_LLnode &arg1, const p_LLnode &arg2)
        {
            //Sort desc
            //return (arg1->data <= arg2->data);
            //Sort asc
            return (arg1->data >= arg2->data);
        }
        
        ///  Build queue by adding new elements to the back of the list
        void addQ(float arg, uint8_t ind)
        {
            p_LLnode tmp =new LLnode(arg, ind);
            
            if (head == nullptr)
                head = tail = tmp;
            else
            {
                tmp->_prev = tail;
                tail->_next = tmp;
                tail = tmp;
            }
        }
        
        ///  Add arg in a list in sorted way - assume list is already sorted
        void addS(float arg, uint8_t ind)
        {
            p_LLnode    tmp=new LLnode(arg, ind),//  Create new node on the heap
                        node = head;            //  Define starting node
            
            siz++;
            //  Traverse the list until we reach the end
            while (node != nullptr)
            {
                //  Sorting logic
                if (sortLogic(tmp, node)) break;
                //  If sorting logic doesn't break the loop move to nex element
                node = node->_next;
            }
            
            /// INSERTION LOGIC
            //  a)Haven't  moved from start - we have new smallest node
            if (node == head)   //Insert before first element
            {
                //  Check if there're any elements at all in the list
                if (head != nullptr) //If yes update head_previous node
                    head->_prev = tmp;
                else //If not update tail node as well 
                    tail = tmp;

                tmp->_next = head;  // Update next node (tmp will become head node)
                head = tmp; // Update head node
            }
            // b)Reached end of the list - insert node after last one
            else if (node == nullptr)   //Insert after last element
            {
                tail->_next = tmp;  // Update next element of current tail
                tmp->_prev = tail;   // Update prev element of new tail element
                tail = tmp;         //Change tail element
            }
            // c)Inserting element in the middle, before 'node'
            else
            {
                tmp->_prev = node->_prev;
                node->_prev->_next = tmp;
                tmp->_next = node;
                node->_prev = tmp;
            }
        }
        

        
        /// Check if the list is empty
        bool empty()
        {
            return (head == nullptr);
        }
        
        ///  Delete & return first element
        float popFront()
        {
            //  Checl if list is empty
            if (LinkedList::empty()) return-1.0;
            //  Check ifthere's only one element in this list
            if (head == tail) tail = nullptr;
            //  Extract data from node before it's deleted
            float retVal = head->data;
            //  Move second node to the first position
            //  If there's only one node next points to nullptr so it's safe
            p_LLnode tmp = head->_next;
            delete head;
            head = tmp;
            if (head != nullptr) head->_prev = nullptr;
            //  Elements are smart pointers, deletion from heap handled internally
            //  when all pointers to it are erased
            siz--;
            return retVal;
        }
        
        /**
         *  Return data in element at index 'index' in list
         *  @brief All nodes before the one have to be traversed so function
         *  completes in O(index)
         */
        float at(uint16_t index)
        {
            uint16_t i = 0;
            p_LLnode node = head;
            
            while ( (i++ < index) && (node->_next != nullptr) )
                node = node->_next;
            
            return node->data;
        }

        void DeleteWhereIndex(uint8_t index)
        {
            p_LLnode node = head;

            while (node->_next != nullptr)
            {
                if (node->index == index)
                    break;
                //  If sorting logic doesn't break the loop move to next element
                node = node->_next;
            }

            //  First node on the list
            if (node == head)
            {
                head = node->_next;
                node->_next->_prev = nullptr;
            }
            //  Last node on the list
            else if (node == tail)
            {
                tail = node->_prev;
                node->_prev->_next = nullptr;
            }
            //  node in the middle
            else
            {
                node->_prev->_next = node->_next;
                node->_next->_prev = node->_prev;
            }

            delete node;
            siz--;
        }

        uint32_t Size()
        {
            return siz;
        }
       
    private:
        p_LLnode head, tail;
        uint32_t siz;
        bool refExists;
};

#endif
