import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from '../css/custom.css';

interface ExpenseItem {
  id: string;
  name: string;
  price: number;
  quantity: number;
  completed: boolean;
}

interface ExpenseCategory {
  id: string;
  name: string;
  items: ExpenseItem[];
  nextDueDate: Date;
}

const RecurringExpensesDashboard: React.FC = () => {
  const [categories, setCategories] = useState<ExpenseCategory[]>([]);
  const [newCategoryName, setNewCategoryName] = useState('');
  const [newItemName, setNewItemName] = useState('');
  const [newItemPrice, setNewItemPrice] = useState('');
  const [newItemQuantity, setNewItemQuantity] = useState('1');
  const [selectedCategory, setSelectedCategory] = useState<string | null>(null);

  // Load data from localStorage on component mount
  useEffect(() => {
    const savedData = localStorage.getItem('recurringExpenses');
    if (savedData) {
      try {
        const parsedData = JSON.parse(savedData);
        // Convert date strings back to Date objects
        const updatedCategories = parsedData.map((category: any) => ({
          ...category,
          nextDueDate: new Date(category.nextDueDate)
        }));
        setCategories(updatedCategories);
      } catch (error) {
        console.error('Error loading recurring expenses:', error);
      }
    } else {
      // Initialize with sample categories
      setCategories([
        {
          id: '1',
          name: 'Groceries',
          items: [],
          nextDueDate: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000) // 1 week from now
        },
        {
          id: '2',
          name: 'Utilities',
          items: [],
          nextDueDate: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000) // 1 week from now
        }
      ]);
    }
  }, []);

  // Save data to localStorage whenever categories change
  useEffect(() => {
    localStorage.setItem('recurringExpenses', JSON.stringify(categories));
  }, [categories]);

  const addCategory = () => {
    if (!newCategoryName.trim()) return;

    const newCategory: ExpenseCategory = {
      id: Date.now().toString(),
      name: newCategoryName.trim(),
      items: [],
      nextDueDate: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000) // 1 month from now
    };

    setCategories([...categories, newCategory]);
    setNewCategoryName('');
  };

  const addItem = () => {
    if (!selectedCategory || !newItemName.trim() || !newItemPrice.trim()) return;

    const price = parseFloat(newItemPrice);
    const quantity = parseInt(newItemQuantity) || 1;

    if (isNaN(price) || price <= 0) {
      alert('Please enter a valid price');
      return;
    }

    const newItem: ExpenseItem = {
      id: Date.now().toString(),
      name: newItemName.trim(),
      price,
      quantity,
      completed: false
    };

    setCategories(categories.map(category =>
      category.id === selectedCategory
        ? { ...category, items: [...category.items, newItem] }
        : category
    ));

    setNewItemName('');
    setNewItemPrice('');
    setNewItemQuantity('1');
  };

  const toggleItemCompletion = (categoryId: string, itemId: string) => {
    setCategories(categories.map(category =>
      category.id === categoryId
        ? {
            ...category,
            items: category.items.map(item =>
              item.id === itemId ? { ...item, completed: !item.completed } : item
            )
          }
        : category
    ));
  };

  const deleteItem = (categoryId: string, itemId: string) => {
    setCategories(categories.map(category =>
      category.id === categoryId
        ? {
            ...category,
            items: category.items.filter(item => item.id !== itemId)
          }
        : category
    ));
  };

  const deleteCategory = (categoryId: string) => {
    setCategories(categories.filter(category => category.id !== categoryId));
    if (selectedCategory === categoryId) {
      setSelectedCategory(null);
    }
  };

  const updateItem = (categoryId: string, itemId: string, updates: Partial<ExpenseItem>) => {
    setCategories(categories.map(category =>
      category.id === categoryId
        ? {
            ...category,
            items: category.items.map(item =>
              item.id === itemId ? { ...item, ...updates } : item
            )
          }
        : category
    ));
  };

  const calculateTotal = () => {
    return categories.reduce((total, category) => {
      const categoryTotal = category.items.reduce((catTotal, item) => {
        if (!item.completed) {
          return catTotal + (item.price * item.quantity);
        }
        return catTotal;
      }, 0);
      return total + categoryTotal;
    }, 0);
  };

  const formatDate = (date: Date) => {
    return date.toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric'
    });
  };

  const totalAmount = calculateTotal();

  return (
    <div className={clsx(styles.recurringExpensesContainer)}>
      <div className={styles.dashboardHeader}>
        <h2>Recurring Expenses Dashboard</h2>
        <p>Track monthly recurring expenses with categories</p>
      </div>

      <div className={styles.expensesSummary}>
        <div className={styles.totalCard}>
          <h3>Total Amount Due</h3>
          <p className={styles.totalAmount}>${totalAmount.toFixed(2)}</p>
        </div>
      </div>

      <div className={styles.addCategorySection}>
        <h3>Add New Category</h3>
        <div className={styles.inputGroup}>
          <input
            type="text"
            value={newCategoryName}
            onChange={(e) => setNewCategoryName(e.target.value)}
            placeholder="Category name (e.g., Groceries)"
            className={styles.inputField}
          />
          <button onClick={addCategory} className={styles.addButton}>
            Add Category
          </button>
        </div>
      </div>

      <div className={styles.addExpenseSection}>
        <h3>Add New Item</h3>
        <div className={styles.inputGroup}>
          <select
            value={selectedCategory || ''}
            onChange={(e) => setSelectedCategory(e.target.value || null)}
            className={styles.selectField}
          >
            <option value="">Select Category</option>
            {categories.map(category => (
              <option key={category.id} value={category.id}>
                {category.name}
              </option>
            ))}
          </select>

          <input
            type="text"
            value={newItemName}
            onChange={(e) => setNewItemName(e.target.value)}
            placeholder="Item name (e.g., Flour)"
            className={styles.inputField}
          />

          <input
            type="number"
            value={newItemPrice}
            onChange={(e) => setNewItemPrice(e.target.value)}
            placeholder="Price ($)"
            min="0"
            step="0.01"
            className={styles.inputField}
          />

          <input
            type="number"
            value={newItemQuantity}
            onChange={(e) => setNewItemQuantity(e.target.value)}
            placeholder="Quantity"
            min="1"
            className={styles.inputField}
          />

          <button onClick={addItem} disabled={!selectedCategory} className={styles.addButton}>
            Add Item
          </button>
        </div>
      </div>

      <div className={styles.categoriesList}>
        {categories.map(category => (
          <div key={category.id} className={styles.categoryCard}>
            <div className={styles.categoryHeader}>
              <h3>{category.name}</h3>
              <div className={styles.dueDate}>
                Next due: {formatDate(category.nextDueDate)}
              </div>
              <button
                onClick={() => deleteCategory(category.id)}
                className={styles.deleteButton}
              >
                Delete Category
              </button>
            </div>

            <div className={styles.itemsList}>
              {category.items.length > 0 ? (
                <table className={styles.itemsTable}>
                  <thead>
                    <tr>
                      <th>Item</th>
                      <th>Price</th>
                      <th>Qty</th>
                      <th>Total</th>
                      <th>Status</th>
                      <th>Actions</th>
                    </tr>
                  </thead>
                  <tbody>
                    {category.items.map(item => (
                      <tr key={item.id} className={item.completed ? styles.completedRow : ''}>
                        <td>
                          <input
                            type="text"
                            value={item.name}
                            onChange={(e) => updateItem(category.id, item.id, { name: e.target.value })}
                            className={styles.itemInput}
                          />
                        </td>
                        <td>
                          <input
                            type="number"
                            value={item.price}
                            onChange={(e) => updateItem(category.id, item.id, { price: parseFloat(e.target.value) || 0 })}
                            min="0"
                            step="0.01"
                            className={styles.itemInput}
                          />
                        </td>
                        <td>
                          <input
                            type="number"
                            value={item.quantity}
                            onChange={(e) => updateItem(category.id, item.id, { quantity: parseInt(e.target.value) || 1 })}
                            min="1"
                            className={styles.itemInput}
                          />
                        </td>
                        <td>${(item.price * item.quantity).toFixed(2)}</td>
                        <td>
                          <label className={styles.checkboxLabel}>
                            <input
                              type="checkbox"
                              checked={item.completed}
                              onChange={() => toggleItemCompletion(category.id, item.id)}
                            />
                            {item.completed ? 'Completed' : 'Pending'}
                          </label>
                        </td>
                        <td>
                          <button
                            onClick={() => deleteItem(category.id, item.id)}
                            className={styles.deleteButtonSmall}
                          >
                            Delete
                          </button>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              ) : (
                <p className={styles.noItems}>No items in this category yet.</p>
              )}
            </div>

            <div className={styles.categoryTotal}>
              Category Total: $
              {category.items
                .filter(item => !item.completed)
                .reduce((sum, item) => sum + (item.price * item.quantity), 0)
                .toFixed(2)}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default RecurringExpensesDashboard;